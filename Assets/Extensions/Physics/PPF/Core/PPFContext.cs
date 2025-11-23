using System;
using Unity.Collections;
using Unity.Mathematics;

namespace mehmetsrl.Physics.PPF.Core
{
    // Core data context holding simulation state (particles, rigid bodies, proxies)
    public class PPFContext : IDisposable
    {
        public int ParticleCapacity { get; private set; }
        public int BodyCapacity { get; private set; }

        // Particles / proxies (generic proxies representing geometry)
        public NativeList<float3> Position;
        public NativeList<float3> PredictedPosition;
        public NativeList<float3> Velocity;
        public NativeList<float> Radius;
        public NativeList<int> ProxyBodyID; // link to body
        public NativeList<float3> LocalOffset; // local offset for proxies belonging to rigid bodies

        // Rigid bodies
        public NativeList<float3> BodyPosition;
        public NativeList<quaternion> BodyRotation;
        public NativeList<float3> BodyVelocity;
        public NativeList<float3> BodyAngularVelocity;
        public NativeList<float> BodyInverseMass;
        public NativeList<float3> BodyInverseInertia;
        public NativeList<int2> BodyProxySlices; // start,count in proxy list

        // Triangle topology (indices reference proxies)
        public NativeList<int3> Triangles;
        public NativeList<int> TriangleBodyIDs;

        // Contact / TOI buffers (transient per-step)
        public NativeList<PPFContact> Contacts;
        public NativeList<TOIEvent> TOIEvents;

        // Spatial maps
        public NativeParallelMultiHashMap<int, int> SpatialMap;
        public NativeParallelMultiHashMap<int, int> TriangleSpatialMap;

        public PPFContext(int particleCap = 16000, int bodyCap = 1024)
        {
            ParticleCapacity = particleCap;
            BodyCapacity = bodyCap;
            Allocator alloc = Allocator.Persistent;

            Position = new NativeList<float3>(particleCap, alloc);
            PredictedPosition = new NativeList<float3>(particleCap, alloc);
            Velocity = new NativeList<float3>(particleCap, alloc);
            Radius = new NativeList<float>(particleCap, alloc);
            ProxyBodyID = new NativeList<int>(particleCap, alloc);
            LocalOffset = new NativeList<float3>(particleCap, alloc);

            BodyPosition = new NativeList<float3>(bodyCap, alloc);
            BodyRotation = new NativeList<quaternion>(bodyCap, alloc);
            BodyVelocity = new NativeList<float3>(bodyCap, alloc);
            BodyAngularVelocity = new NativeList<float3>(bodyCap, alloc);
            BodyInverseMass = new NativeList<float>(bodyCap, alloc);
            BodyInverseInertia = new NativeList<float3>(bodyCap, alloc);
            BodyProxySlices = new NativeList<int2>(bodyCap, alloc);

            Triangles = new NativeList<int3>(particleCap, alloc);
            TriangleBodyIDs = new NativeList<int>(particleCap, alloc);

            Contacts = new NativeList<PPFContact>(1024, alloc);
            TOIEvents = new NativeList<TOIEvent>(256, alloc);

            int mapCap = math.max(250000, particleCap * 20);
            SpatialMap = new NativeParallelMultiHashMap<int, int>(mapCap, alloc);
            TriangleSpatialMap = new NativeParallelMultiHashMap<int, int>(mapCap, alloc);
        }

        public void ClearTransient()
        {
            Contacts.Clear();
            TOIEvents.Clear();
            SpatialMap.Clear();
            TriangleSpatialMap.Clear();
        }

        public void ClearAll()
        {
            Position.Clear();
            PredictedPosition.Clear();
            Velocity.Clear();
            Radius.Clear();
            ProxyBodyID.Clear();
            LocalOffset.Clear();
            BodyPosition.Clear();
            BodyRotation.Clear();
            BodyVelocity.Clear();
            BodyAngularVelocity.Clear();
            BodyInverseMass.Clear();
            BodyInverseInertia.Clear();
            BodyProxySlices.Clear();
            Triangles.Clear();
            TriangleBodyIDs.Clear();
            ClearTransient();
        }

        public void Dispose()
        {
            if (Position.IsCreated) Position.Dispose();
            if (PredictedPosition.IsCreated) PredictedPosition.Dispose();
            if (Velocity.IsCreated) Velocity.Dispose();
            if (Radius.IsCreated) Radius.Dispose();
            if (ProxyBodyID.IsCreated) ProxyBodyID.Dispose();
            if (LocalOffset.IsCreated) LocalOffset.Dispose();

            if (BodyPosition.IsCreated) BodyPosition.Dispose();
            if (BodyRotation.IsCreated) BodyRotation.Dispose();
            if (BodyVelocity.IsCreated) BodyVelocity.Dispose();
            if (BodyAngularVelocity.IsCreated) BodyAngularVelocity.Dispose();
            if (BodyInverseMass.IsCreated) BodyInverseMass.Dispose();
            if (BodyInverseInertia.IsCreated) BodyInverseInertia.Dispose();
            if (BodyProxySlices.IsCreated) BodyProxySlices.Dispose();

            if (Triangles.IsCreated) Triangles.Dispose();
            if (TriangleBodyIDs.IsCreated) TriangleBodyIDs.Dispose();

            if (Contacts.IsCreated) Contacts.Dispose();
            if (TOIEvents.IsCreated) TOIEvents.Dispose();

            if (SpatialMap.IsCreated) SpatialMap.Dispose();
            if (TriangleSpatialMap.IsCreated) TriangleSpatialMap.Dispose();
        }
    }
}