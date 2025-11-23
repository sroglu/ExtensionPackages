using UnityEngine;
using System.Collections.Generic;
using mehmetsrl.Extensions.VerletSystem.Core;
using mehmetsrl.Extensions.VerletSystem.Solver;
using Sirenix.OdinInspector;

namespace mehmetsrl.Extensions.VerletSystem
{
    public class VerletRope : MonoBehaviour
    {
        [Header("Dependencies")]
        [SerializeField] private MonoBehaviour visualizerComponent; 
        private IRopeVisualizer _visualizer;
        private IVerletSolver _solver;

        [Header("Topology")]
        public Transform startAnchor;
        public Transform endAnchor;

        [Tooltip("Nodes per meter.")]
        [SerializeField, OnValueChanged(nameof(ForceUpdateSettings))]
        private float segmentCountLengthRatio = 10f;

        [Header("Dynamic Update Settings")]
        [SerializeField] private bool autoUpdateLength = true;
        [SerializeField] private bool autoUpdateSegmentCount = true;
        [SerializeField] private bool useHighQualityResampling = true;

        [Header("Resampling Strategy")]
        [Tooltip("If true, the system calculates which anchor moved more and adds nodes there.")]
        [SerializeField] 
        private bool useMotionBasedStrategy = true;

        [Tooltip("Used if Motion Based Strategy is False.")]
        [SerializeField, HideIf(nameof(useMotionBasedStrategy))]
        private ResampleStrategy fixedStrategy = ResampleStrategy.Uniform;

        [Header("State (Read Only)")]
        [SerializeField, ReadOnly] private int segmentCount = 50;
        [SerializeField, ReadOnly] private float totalLength = 5.0f;
        [SerializeField, ReadOnly] private bool isInitialized = false;

        // Configuration Data
        [SerializeField] 
        private RopePhysicsSettings physicsSettings;

        // Runtime Data
        private List<VerletNode> _nodes;
        private float _nominalSegmentLength;
        private float _lastRatio;
        
        // Motion Tracking
        private Vector3 _prevStartPos;
        private Vector3 _prevEndPos;

        // Public Access
        public List<VerletNode> Nodes => _nodes;

        private void Awake()
        {
            if (visualizerComponent is IRopeVisualizer vis) _visualizer = vis;
            else _visualizer = GetComponent<IRopeVisualizer>();
            
            _lastRatio = segmentCountLengthRatio;
        }

        private void Update()
        {
            if (!isInitialized || (!autoUpdateLength && !autoUpdateSegmentCount)) return;

            // Check movement
            bool startMoved = startAnchor && startAnchor.hasChanged;
            bool endMoved = endAnchor && endAnchor.hasChanged;
            bool ratioChanged = Mathf.Abs(_lastRatio - segmentCountLengthRatio) > 0.001f;

            if (startMoved || endMoved || ratioChanged)
            {
                UpdateConfiguration();
                
                // Update history
                if(startAnchor) {
                     startAnchor.hasChanged = false;
                     _prevStartPos = startAnchor.position;
                }
                if(endAnchor) {
                     endAnchor.hasChanged = false;
                     _prevEndPos = endAnchor.position;
                }
                _lastRatio = segmentCountLengthRatio;
            }
        }
        
        private void ForceUpdateSettings() { if(Application.isPlaying && isInitialized) UpdateConfiguration(); }

        private void UpdateConfiguration()
        {
            if (!startAnchor || !endAnchor) return;

            float dist = Vector3.Distance(startAnchor.position, endAnchor.position);

            // 1. Determine Strategy based on Motion
            ResampleStrategy currentStrategy = fixedStrategy;

            if (useMotionBasedStrategy)
            {
                float deltaStart = Vector3.Distance(startAnchor.position, _prevStartPos);
                float deltaEnd = Vector3.Distance(endAnchor.position, _prevEndPos);

                // Sensitivity threshold
                if (deltaStart > 0.001f || deltaEnd > 0.001f)
                {
                    if (deltaStart > deltaEnd)
                        currentStrategy = ResampleStrategy.AddAtHead;
                    else
                        currentStrategy = ResampleStrategy.AddAtTail;
                }
                else
                {
                    // No significant movement, default to uniform
                    currentStrategy = ResampleStrategy.Uniform;
                }
            }

            // 2. Update Length
            if (autoUpdateLength)
            {
                totalLength = dist;
                RecalculateConstraints();
            }

            // 3. Update Topology
            if (autoUpdateSegmentCount)
            {
                int targetCount = Mathf.Max(2, Mathf.RoundToInt(totalLength * segmentCountLengthRatio));
                
                if (targetCount != segmentCount)
                {
                    PerformResampling(targetCount, currentStrategy);
                }
            }
        }

        private void PerformResampling(int newCount, ResampleStrategy strategy)
        {
            if (_nodes == null || _nodes.Count < 2)
            {
                segmentCount = newCount;
                BuildRopeFromScratch();
                return;
            }

            bool endStatic = (endAnchor != null);
            
            // Pass strategy to helper
            _nodes = RopeResampler.Resample(_nodes, newCount, physicsSettings, useHighQualityResampling, endStatic, strategy);
            
            segmentCount = newCount;
            RecalculateConstraints();
            
            if (_solver != null) _solver.SyncNodes(_nodes, physicsSettings.collisionLayers);
            UpdateVisualizer();
        }

        private void BuildRopeFromScratch()
        {
            Vector3 s = startAnchor ? startAnchor.position : transform.position;
            Vector3 e = endAnchor ? endAnchor.position : s + Vector3.down * totalLength;
            bool endStatic = (endAnchor != null);

            _nodes = RopeResampler.CreateStraightLine(s, e, segmentCount, physicsSettings, endStatic);
            
            // Init previous positions for motion detection
            _prevStartPos = s;
            _prevEndPos = e;

            RecalculateConstraints();
            UpdateVisualizer();
            
            if(startAnchor) startAnchor.hasChanged = false;
            if(endAnchor) endAnchor.hasChanged = false;
        }

        private void RecalculateConstraints()
        {
            if (segmentCount < 2) return;
            _nominalSegmentLength = totalLength / (segmentCount - 1);
        }

        // --- INITIALIZATION ---

#if UNITY_EDITOR
        [Button("Initialize (Auto Find Solver)")]
        public void InitializeRuntimeTest()
        {
            if (!Application.isPlaying) return;
            var solver = FindObjectOfType<mehmetsrl.Extensions.PPF.Solvers.VerletStandardSolver>();
            if (solver == null) { Debug.LogError("No PPFStandardSolver in scene!"); return; }
            Initialize(solver);
        }
#endif


        public void Initialize(IVerletSolver solver)
        {
            if (solver == null) throw new System.ArgumentNullException(nameof(solver));
            
            _solver = solver;
            isInitialized = false;

            if (segmentCount < 2) segmentCount = 2;
            
            BuildRopeFromScratch();
            isInitialized = true;
        }

        // --- PHYSICS LOOP ---

        private void FixedUpdate()
        {
            if (!isInitialized || _nodes == null || _nodes.Count == 0) return;
            float dt = Time.fixedDeltaTime;

            VerletSimulator.Step(_nodes, physicsSettings, dt);

            for (int i = 0; i < physicsSettings.constraintIterations; i++)
            {
                Vector3? s = startAnchor ? startAnchor.position : null;
                Vector3? e = endAnchor ? endAnchor.position : null;

                VerletSimulator.ApplyConstraints(_nodes, _nominalSegmentLength, physicsSettings.stiffness, s, e);

                if (_solver != null)
                {
                    // Note: Pass rope specific layers if needed (physicsSettings.collisionLayers)
                    _solver.SyncNodes(_nodes, physicsSettings.collisionLayers);
                    _solver.Solve(dt);
                    _solver.RetrieveNodes(_nodes);
                }
            }
        }

        private void LateUpdate()
        {
            if (isInitialized) UpdateVisualizer();
        }

        private void UpdateVisualizer()
        {
            _visualizer?.UpdateVisualization(_nodes, physicsSettings.thickness);
        }

        // --- API ---
        
        public void SetColorGradient(List<RopeColorSection> segments) => _visualizer?.SetColorSegments(segments);
        public void SetMaterial(Material material) => _visualizer?.SetMaterial(material);

        public void ChangeLength(float amount)
        {
            if (!isInitialized) return;

            totalLength += amount;
            if (totalLength < 0.1f) totalLength = 0.1f;

            RecalculateConstraints();

            if (autoUpdateSegmentCount)
            {
                int targetCount = Mathf.Max(2, Mathf.RoundToInt(totalLength * segmentCountLengthRatio));
                if (targetCount != segmentCount)
                {
                    // Manual length change implies Uniform usually, or handled by logic
                    PerformResampling(targetCount, ResampleStrategy.Uniform);
                }
            }
        }

        public void Teleport(Vector3 start, Vector3 end)
        {
            if (!isInitialized) { Debug.LogWarning("Rope not initialized!"); return; }
            
            bool endStatic = (endAnchor != null);
            _nodes = RopeResampler.CreateStraightLine(start, end, _nodes.Count, physicsSettings, endStatic);
            
            _prevStartPos = start;
            _prevEndPos = end;
        }
        
        public void ResetParticles()
        {
            if (isInitialized && startAnchor && endAnchor) 
                Teleport(startAnchor.position, endAnchor.position);
        }
    }
}