using System;
using Unity.Collections;
using Unity.Mathematics;

namespace mehmetsrl.Physics.XPBD.Core
{
    // The core data container for the physics engine.
    // Holds simulation state in Structure of Arrays (SoA) layout.
    public class PhysicsContext : IDisposable
    {
        public int Capacity { get; private set; }
        public int ParticleCount => CurrentPosition.Length;

        // --- State Arrays ---
        public NativeList<float3> CurrentPosition;
        public NativeList<float3> PredictedPosition;
        public NativeList<float3> PreviousPosition;
        public NativeList<float3> Velocity;
        public NativeList<float> InverseMass;
        public NativeList<float> Radius;

        // Identity & Material
        public NativeList<int> BodyIDs;
        public NativeList<float3> PositionLockMask;
        public NativeList<float> CalculatedLocalStiffness;

        // --- Topology (Constraints) ---
        public NativeList<int2> DistanceConstraints;
        public NativeList<float> RestLengths;
        public NativeList<float> Compliances;

        // --- Mesh Topology (Rigid Bodies) ---
        public NativeList<int3> Triangles;

        // --- Broadphase Maps ---
        public NativeParallelMultiHashMap<int, int> SpatialMap;
        public NativeParallelMultiHashMap<int, int> TriangleSpatialMap;

        public PhysicsContext(int initialCapacity = 10000)
        {
            Capacity = initialCapacity;
            Allocator alloc = Allocator.Persistent;

            CurrentPosition = new NativeList<float3>(Capacity, alloc);
            PredictedPosition = new NativeList<float3>(Capacity, alloc);
            PreviousPosition = new NativeList<float3>(Capacity, alloc);
            Velocity = new NativeList<float3>(Capacity, alloc);
            InverseMass = new NativeList<float>(Capacity, alloc);
            Radius = new NativeList<float>(Capacity, alloc);

            BodyIDs = new NativeList<int>(Capacity, alloc);
            PositionLockMask = new NativeList<float3>(Capacity, alloc);
            CalculatedLocalStiffness = new NativeList<float>(Capacity, alloc);

            DistanceConstraints = new NativeList<int2>(Capacity, alloc);
            RestLengths = new NativeList<float>(Capacity, alloc);
            Compliances = new NativeList<float>(Capacity, alloc);

            Triangles = new NativeList<int3>(Capacity, alloc);

            // --- FIX: ROBUST MAP CAPACITY ---
            // Triangles can span multiple cells, causing the map to fill up quickly.
            // We use a large multiplier (x30) or a minimum safe size (100k) to prevent crashes.
            int minCapacity = 250000;
            int calculatedCapacity = initialCapacity * 50;
            int finalCapacity = math.max(minCapacity, calculatedCapacity);

            SpatialMap = new NativeParallelMultiHashMap<int, int>(finalCapacity, alloc);
            TriangleSpatialMap = new NativeParallelMultiHashMap<int, int>(finalCapacity, alloc);
        }

        public void Clear()
        {
            CurrentPosition.Clear();
            PredictedPosition.Clear();
            PreviousPosition.Clear();
            Velocity.Clear();
            InverseMass.Clear();
            Radius.Clear();
            BodyIDs.Clear();
            PositionLockMask.Clear();
            CalculatedLocalStiffness.Clear();

            DistanceConstraints.Clear();
            RestLengths.Clear();
            Compliances.Clear();

            Triangles.Clear();
            SpatialMap.Clear();
            TriangleSpatialMap.Clear();
        }

        public void Dispose()
        {
            if (CurrentPosition.IsCreated) CurrentPosition.Dispose();
            if (PredictedPosition.IsCreated) PredictedPosition.Dispose();
            if (PreviousPosition.IsCreated) PreviousPosition.Dispose();
            if (Velocity.IsCreated) Velocity.Dispose();
            if (InverseMass.IsCreated) InverseMass.Dispose();
            if (Radius.IsCreated) Radius.Dispose();
            if (BodyIDs.IsCreated) BodyIDs.Dispose();
            if (PositionLockMask.IsCreated) PositionLockMask.Dispose();
            if (CalculatedLocalStiffness.IsCreated) CalculatedLocalStiffness.Dispose();

            if (DistanceConstraints.IsCreated) DistanceConstraints.Dispose();
            if (RestLengths.IsCreated) RestLengths.Dispose();
            if (Compliances.IsCreated) Compliances.Dispose();

            if (Triangles.IsCreated) Triangles.Dispose();
            if (SpatialMap.IsCreated) SpatialMap.Dispose();
            if (TriangleSpatialMap.IsCreated) TriangleSpatialMap.Dispose();
        }
    }
}