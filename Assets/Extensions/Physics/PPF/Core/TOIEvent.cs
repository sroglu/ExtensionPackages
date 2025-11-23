using Unity.Mathematics;

namespace mehmetsrl.Physics.PPF.Core
{

    // Time-of-impact event useful for conservative advancement
    public struct TOIEvent
    {
        public int ProxyIndex;
        public int OtherIndex;
        public float TOI;
        public float3 Normal;
    }
}