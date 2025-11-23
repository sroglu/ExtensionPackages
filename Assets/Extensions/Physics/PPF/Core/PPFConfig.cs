using System;
using Unity.Mathematics;

namespace mehmetsrl.Physics.PPF.Core
{
    [Serializable]
    public struct PPFConfig
    {
        public float3 Gravity;
        public int SolverIterations; // internal solver iterations for substeps
        public float GridCellSize;
        public float TimeStepMultiplier; // for substepping or conservative advancement

        public static PPFConfig Default => new PPFConfig
        {
            Gravity = new float3(0, -9.81f, 0),
            SolverIterations = 8,
            GridCellSize = 0.5f,
            TimeStepMultiplier = 1.0f
        };
    }
}