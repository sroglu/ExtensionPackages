using Unity.Mathematics;
using System;

namespace mehmetsrl.Physics.XPBD.Core
{
    [Serializable]
    public struct PhysicsConfig
    {
        public float3 Gravity;
        public int SolverIterations;
        public float GridCellSize; // Broadphase cell size

        // Ando's Elasticity-Inclusive Ratio
        // 1.0 = Barrier is as stiff as the object. 
        // 10.0 = Barrier is 10x stiffer.
        public float BarrierStiffnessRatio;

        public static PhysicsConfig Default => new PhysicsConfig
        {
            Gravity = PhysicsConstants.EarthGravity,
            SolverIterations = 10,
            GridCellSize = 0.5f,
            BarrierStiffnessRatio = 1.0f
        };
    }
}