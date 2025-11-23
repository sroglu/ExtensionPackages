using System;
using Unity.Collections;
using Unity.Mathematics;

namespace mehmetsrl.Physics.XPBD.Core
{
    public class PhysicsContext : IDisposable
    {
        public int ParticleCapacity { get; private set; }
        public int BodyCapacity { get; private set; }

        public int ParticleCount => CurrentPosition.Length;
        public int BodyCount => BodyPosition.Length;

        // --- PARTICLE DATA ---
        public NativeList<float3> CurrentPosition;   
        public NativeList<float3> PredictedPosition; 
        public NativeList<float3> PreviousPosition;  
        public NativeList<float3> Velocity;          
        public NativeList<float>  Radius;
        public NativeList<int>    ParticleBodyID; 
        public NativeList<float3> ParticleLocalPosition; 

        // --- RIGID BODY DATA ---
        public NativeList<float3> BodyPosition;
        public NativeList<quaternion> BodyRotation;
        public NativeList<float3> BodyVelocity;
        public NativeList<float3> BodyAngularVelocity;
        public NativeList<float3> BodyPredictedPos;
        public NativeList<quaternion> BodyPredictedRot;
        public NativeList<float>  BodyInverseMass;
        public NativeList<float3> BodyInverseInertia;
        public NativeList<int2>   BodyParticleSlices; 

        // --- TOPOLOGY & MAPS ---
        public NativeList<int2> DistanceConstraints; 
        public NativeList<float> RestLengths;
        public NativeList<float> Compliances;
        public NativeList<float> CalculatedLocalStiffness; 
        
        // Triangle Data
        public NativeList<int3> Triangles; 
        // NEW: Stores which Body owns the triangle to prevent self-collision
        public NativeList<int> TriangleBodyIDs; 

        public NativeParallelMultiHashMap<int, int> SpatialMap;
        public NativeParallelMultiHashMap<int, int> TriangleSpatialMap;

        public PhysicsContext(int particleCap = 10000, int bodyCap = 1000)
        {
            ParticleCapacity = particleCap;
            BodyCapacity = bodyCap;
            Allocator alloc = Allocator.Persistent;

            CurrentPosition = new NativeList<float3>(particleCap, alloc);
            PredictedPosition = new NativeList<float3>(particleCap, alloc);
            PreviousPosition = new NativeList<float3>(particleCap, alloc);
            Velocity = new NativeList<float3>(particleCap, alloc);        
            Radius = new NativeList<float>(particleCap, alloc);
            ParticleBodyID = new NativeList<int>(particleCap, alloc);
            ParticleLocalPosition = new NativeList<float3>(particleCap, alloc);

            BodyPosition = new NativeList<float3>(bodyCap, alloc);
            BodyRotation = new NativeList<quaternion>(bodyCap, alloc);
            BodyVelocity = new NativeList<float3>(bodyCap, alloc);
            BodyAngularVelocity = new NativeList<float3>(bodyCap, alloc);
            BodyPredictedPos = new NativeList<float3>(bodyCap, alloc);
            BodyPredictedRot = new NativeList<quaternion>(bodyCap, alloc);
            BodyInverseMass = new NativeList<float>(bodyCap, alloc);
            BodyInverseInertia = new NativeList<float3>(bodyCap, alloc);
            BodyParticleSlices = new NativeList<int2>(bodyCap, alloc);

            CalculatedLocalStiffness = new NativeList<float>(particleCap, alloc);
            DistanceConstraints = new NativeList<int2>(particleCap, alloc);
            RestLengths = new NativeList<float>(particleCap, alloc);
            Compliances = new NativeList<float>(particleCap, alloc);
            
            Triangles = new NativeList<int3>(particleCap, alloc);
            TriangleBodyIDs = new NativeList<int>(particleCap, alloc); // Init

            int mapCap = math.max(250000, particleCap * 50);
            SpatialMap = new NativeParallelMultiHashMap<int, int>(mapCap, alloc);
            TriangleSpatialMap = new NativeParallelMultiHashMap<int, int>(mapCap, alloc);
        }

        public void Clear()
        {
            CurrentPosition.Clear(); PredictedPosition.Clear(); 
            PreviousPosition.Clear(); Velocity.Clear();
            Radius.Clear(); ParticleBodyID.Clear(); ParticleLocalPosition.Clear();

            BodyPosition.Clear(); BodyRotation.Clear();
            BodyVelocity.Clear(); BodyAngularVelocity.Clear();
            BodyPredictedPos.Clear(); BodyPredictedRot.Clear();
            BodyInverseMass.Clear(); BodyInverseInertia.Clear();
            BodyParticleSlices.Clear();

            CalculatedLocalStiffness.Clear();
            DistanceConstraints.Clear(); RestLengths.Clear(); Compliances.Clear(); 
            
            Triangles.Clear();
            TriangleBodyIDs.Clear(); // Clear
            
            SpatialMap.Clear(); TriangleSpatialMap.Clear();
        }

        public void Dispose()
        {
            if (CurrentPosition.IsCreated) CurrentPosition.Dispose();
            if (PredictedPosition.IsCreated) PredictedPosition.Dispose();
            if (PreviousPosition.IsCreated) PreviousPosition.Dispose();
            if (Velocity.IsCreated) Velocity.Dispose();
            if (Radius.IsCreated) Radius.Dispose();
            if (ParticleBodyID.IsCreated) ParticleBodyID.Dispose();
            if (ParticleLocalPosition.IsCreated) ParticleLocalPosition.Dispose();

            if (BodyPosition.IsCreated) BodyPosition.Dispose();
            if (BodyRotation.IsCreated) BodyRotation.Dispose();
            if (BodyVelocity.IsCreated) BodyVelocity.Dispose();
            if (BodyAngularVelocity.IsCreated) BodyAngularVelocity.Dispose();
            if (BodyPredictedPos.IsCreated) BodyPredictedPos.Dispose();
            if (BodyPredictedRot.IsCreated) BodyPredictedRot.Dispose();
            if (BodyInverseMass.IsCreated) BodyInverseMass.Dispose();
            if (BodyInverseInertia.IsCreated) BodyInverseInertia.Dispose();
            if (BodyParticleSlices.IsCreated) BodyParticleSlices.Dispose();

            if (CalculatedLocalStiffness.IsCreated) CalculatedLocalStiffness.Dispose();
            if (DistanceConstraints.IsCreated) DistanceConstraints.Dispose();
            if (RestLengths.IsCreated) RestLengths.Dispose();
            if (Compliances.IsCreated) Compliances.Dispose();
            
            if (Triangles.IsCreated) Triangles.Dispose();
            if (TriangleBodyIDs.IsCreated) TriangleBodyIDs.Dispose(); // Dispose

            if (SpatialMap.IsCreated) SpatialMap.Dispose();
            if (TriangleSpatialMap.IsCreated) TriangleSpatialMap.Dispose();
        }
    }
}