using Unity.Mathematics;

namespace mehmetsrl.Physics.PPF.Core
{
    // Contact types used by PPF
    public enum ContactType : byte
    {
        ParticleParticle = 0,
        ParticleTriangle = 1,
        BodyBody = 2
    }

    // Renamed contact struct as requested
    public struct PPFContact
    {
        public int A; // particle / proxy index or body proxy index
        public int B; // particle / triangle index or other proxy
        public ContactType Type;
        public float3 Normal; // pointing from B to A (A - B)
        public float Penetration; // negative if penetrating (distance < 0)
        public float TimeOfImpact; // [0..1] fraction inside timestep (for TOI)
    }
}