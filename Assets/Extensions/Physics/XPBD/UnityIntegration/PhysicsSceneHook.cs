using UnityEngine;
using Unity.Mathematics;
using mehmetsrl.Physics.XPBD.Core;

namespace mehmetsrl.Physics.XPBD.UnityIntegration
{
    // Set execution order to ensure the engine initializes before bodies try to register.
    [DefaultExecutionOrder(-100)]
    public class PhysicsSceneHook : MonoBehaviour
    {
        // Public access to the Core Engine
        public PhysicsEngine World { get; private set; }

        [Header("Simulation Settings")] [Tooltip("Global gravity vector (m/s^2).")]
        public Vector3 Gravity = new Vector3(0, -9.81f, 0);

        [Tooltip("Sub-steps per frame. Higher = More stable but expensive.")] [Range(1, 50)]
        public int SolverIterations = 10;

        [Tooltip("Size of the spatial hash cells. Should be > 2 * MaxParticleRadius.")]
        public float GridCellSize = 0.5f;

        [Header("Elasticity Settings (Ando's Barrier)")]
        [Tooltip("Global multiplier for barrier stiffness relative to object stiffness.")]
        public float BarrierStiffnessRatio = 1.0f;

        void Awake()
        {
            // 1. Create Config
            var config = PhysicsConfig.Default;
            config.Gravity = Gravity;
            config.SolverIterations = SolverIterations;
            config.GridCellSize = GridCellSize;
            config.BarrierStiffnessRatio = BarrierStiffnessRatio;

            // 2. Initialize Pure C# Engine
            World = new PhysicsEngine(config);
        }

        void FixedUpdate()
        {
            if (World != null)
            {
                // Sync config runtime to allow Editor tweaking
                World.Config.Gravity = Gravity;
                World.Config.SolverIterations = SolverIterations;
                World.Config.GridCellSize = GridCellSize;
                World.Config.BarrierStiffnessRatio = BarrierStiffnessRatio;

                // 3. Step Simulation
                World.Step(Time.fixedDeltaTime);
            }
        }

        // Optional: Ensure jobs are done before rendering
        public void CompleteJobs()
        {
            // The Step() method currently completes all handles, 
            // but this is good practice for future async implementations.
        }

        void OnDestroy()
        {
            // 4. Cleanup Native Arrays
            World?.Dispose();
        }
    }
}