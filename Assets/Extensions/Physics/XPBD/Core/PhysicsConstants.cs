using Unity.Mathematics;

namespace mehmetsrl.Physics.XPBD.Core
{
    public static class PhysicsConstants
    {
        public const float EarthGravityMagnitude = 9.81f;
        public static readonly float3 EarthGravity = new float3(0, -EarthGravityMagnitude, 0);

        public const float Epsilon = 1e-9f;
        public const float MaxStiffness = 1e9f;
    }
}