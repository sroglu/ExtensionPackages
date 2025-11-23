using UnityEngine;
using Unity.Mathematics;
using mehmetsrl.Physics.XPBD.Core;

namespace mehmetsrl.Physics.XPBD.UnityIntegration
{
    // Set execution order to ensure the engine initializes before bodies try to register.
    [DefaultExecutionOrder(-100)]
    public class PhysicsSceneHook : MonoBehaviour
    {
        public PhysicsEngine World { get; private set; }

        [Header("Simulation Settings")] public Vector3 Gravity = new Vector3(0, -9.81f, 0);
        [Range(1, 30)] public int SolverIterations = 20; // Lowered default iteration for better FPS
        public float GridCellSize = 0.5f;

        [Header("Elasticity Settings")]
        [Tooltip("Global multiplier for barrier stiffness relative to object stiffness.")]
        public float BarrierStiffnessRatio = 1.0f;

        [Header("Capacity")] public int MaxParticles = 50000;
        public int MaxRigidBodies = 1000;


        void Awake()
        {
            // 1. Prepare Configuration
            var config = PhysicsConfig.Default;
            config.Gravity = (float3)Gravity;
            config.SolverIterations = SolverIterations;
            config.GridCellSize = GridCellSize;
            config.BarrierStiffnessRatio = BarrierStiffnessRatio;

            // 2. Initialize the Engine (RBD API)
            World = new PhysicsEngine(config, MaxParticles, MaxRigidBodies);
        }

        void FixedUpdate()
        {
            if (World != null)
            {
                // Sync configuration
                World.Config.Gravity = (float3)Gravity;
                World.Config.SolverIterations = SolverIterations;
                World.Config.GridCellSize = GridCellSize;
                World.Config.BarrierStiffnessRatio = BarrierStiffnessRatio;

                // 3. Step Simulation
                World.Step(Time.fixedDeltaTime);
            }
        }

        void OnDestroy()
        {
            // Cleanup Native Arrays
            World?.Dispose();
        }
    }
}