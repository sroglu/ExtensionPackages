using Unity.Jobs;
using Unity.Mathematics;
using Unity.Collections;

namespace mehmetsrl.Physics.XPBD.Core
{
    public class PhysicsEngine
    {
        private readonly PhysicsContext _context;
        private SpatialGrid _grid;
        
        public PhysicsConfig Config;
        public PhysicsContext Context => _context;

        public PhysicsEngine(int maxParticles = 10000, int maxBodies = 1000)
        {
            Config = PhysicsConfig.Default;
            _context = new PhysicsContext(maxParticles, maxBodies);
            _grid = new SpatialGrid(Config.GridCellSize);
        }

        public PhysicsEngine(PhysicsConfig config, int maxParticles = 10000, int maxBodies = 1000)
        {
            Config = config;
            _context = new PhysicsContext(maxParticles, maxBodies);
            _grid = new SpatialGrid(Config.GridCellSize);
        }

        // --- API ---

        public int AddBody(float3 position, quaternion rotation, float mass, float3 inverseInertia, float3 angularVelocity)
        {
            int index = _context.BodyPosition.Length;
            _context.BodyPosition.Add(position);
            _context.BodyRotation.Add(rotation);
            _context.BodyPredictedPos.Add(position);
            _context.BodyPredictedRot.Add(rotation);
            _context.BodyVelocity.Add(float3.zero);
            _context.BodyAngularVelocity.Add(angularVelocity);
            
            float invMass = mass > PhysicsConstants.Epsilon ? 1.0f / mass : 0.0f;
            _context.BodyInverseMass.Add(invMass);
            _context.BodyInverseInertia.Add(inverseInertia);

            // Initialize Slice
            int currentTotal = _context.CurrentPosition.Length;
            _context.BodyParticleSlices.Add(new int2(currentTotal, 0));

            return index;
        }

        public int AddCollisionProxy(float3 localOffset, float radius, int bodyIndex)
        {
            int index = _context.CurrentPosition.Length;
            _context.CurrentPosition.Add(float3.zero); 
            _context.PredictedPosition.Add(float3.zero);
            _context.PreviousPosition.Add(float3.zero);
            _context.Velocity.Add(float3.zero);
            _context.Radius.Add(radius);
            _context.ParticleBodyID.Add(bodyIndex);
            _context.ParticleLocalPosition.Add(localOffset);
            _context.CalculatedLocalStiffness.Add(0.0f);

            // Update Slice
            int2 slice = _context.BodyParticleSlices[bodyIndex];
            slice.y += 1;
            _context.BodyParticleSlices[bodyIndex] = slice;
            
            return index;
        }

        // FIX: Added bodyIndex parameter to track ownership
        public void AddTriangle(int a, int b, int c, int bodyIndex)
        {
            _context.Triangles.Add(new int3(a, b, c));
            _context.TriangleBodyIDs.Add(bodyIndex); // Store ID
        }

        public void AddConstraint(int indexA, int indexB, float restLength, float compliance)
        {
            _context.DistanceConstraints.Add(new int2(indexA, indexB));
            _context.RestLengths.Add(restLength);
            _context.Compliances.Add(compliance);
        }

        public void Clear()
        {
            _context.Clear();
        }

        // --- LOOP ---

        public void Step(float deltaTime)
        {
            int particleCount = _context.ParticleCount;
            int bodyCount = _context.BodyCount;
            if (particleCount == 0 && bodyCount == 0) return;

            float dt = deltaTime / Config.SolverIterations;
            if (math.abs(_grid.CellSize - Config.GridCellSize) > 1e-4f)
                _grid = new SpatialGrid(Config.GridCellSize);

            JobHandle handle = default;
            for (int i = 0; i < Config.SolverIterations; i++)
            {
                handle = RunSubStep(dt, handle, particleCount, bodyCount);
            }
            handle.Complete();
        }

        private JobHandle RunSubStep(float dt, JobHandle dependency, int particleCount, int bodyCount)
        {
            // 1. Prediction
            var rbPred = new PhysicsJobs.RigidBodyPredictionJob
            {
                PredictedPos = _context.BodyPredictedPos.AsArray(),
                PredictedRot = _context.BodyPredictedRot.AsArray(),
                Velocity = _context.BodyVelocity.AsArray(),
                AngularVelocity = _context.BodyAngularVelocity.AsArray(),
                Position = _context.BodyPosition.AsArray(),
                Rotation = _context.BodyRotation.AsArray(),
                InvMass = _context.BodyInverseMass.AsArray(),
                Gravity = Config.Gravity,
                DeltaTime = dt
            };
            dependency = rbPred.Schedule(bodyCount, 64, dependency);

            // 2. Sync Proxies
            var syncJob = new PhysicsJobs.SyncProxiesJob
            {
                PredictedParticlePos = _context.PredictedPosition.AsArray(),
                ParticleBodyID = _context.ParticleBodyID.AsArray(),
                LocalPos = _context.ParticleLocalPosition.AsArray(),
                BodyPos = _context.BodyPredictedPos.AsArray(),
                BodyRot = _context.BodyPredictedRot.AsArray()
            };
            dependency = syncJob.Schedule(particleCount, 64, dependency);

            // 3. Broadphase
            dependency.Complete(); 
            _context.SpatialMap.Clear();
            _context.TriangleSpatialMap.Clear();
            
            var buildMap = new PhysicsJobs.BuildSpatialMapJob 
            {
                MapWriter = _context.SpatialMap.AsParallelWriter(),
                Positions = _context.PredictedPosition.AsArray(),
                Grid = _grid
            };
            dependency = buildMap.Schedule(particleCount, 64, default);

            var buildTriMap = new PhysicsJobs.BuildTriangleGridJob 
            {
                MapWriter = _context.TriangleSpatialMap.AsParallelWriter(),
                Triangles = _context.Triangles.AsArray(),
                Positions = _context.PredictedPosition.AsArray(),
                Grid = _grid,
                Margin = 0.05f 
            };
            dependency = buildTriMap.Schedule(_context.Triangles.Length, 64, dependency);

            // 4. Barriers (Body-Centric)
            var rbBarrier = new PhysicsJobs.RigidBodyBarrierJob
            {
                BodyPos = _context.BodyPredictedPos.AsArray(),
                BodyRot = _context.BodyPredictedRot.AsArray(),
                BodyParticleSlices = _context.BodyParticleSlices.AsArray(),
                
                ParticlePos = _context.PredictedPosition.AsArray(),
                ParticleRadius = _context.Radius.AsArray(),
                ParticleBodyID = _context.ParticleBodyID.AsArray(),
                
                BodyInvMass = _context.BodyInverseMass.AsArray(),
                BodyInvInertia = _context.BodyInverseInertia.AsArray(),
                
                Triangles = _context.Triangles.AsArray(),
                TriangleBodyIDs = _context.TriangleBodyIDs.AsArray(), // FIX: Pass ID Array
                
                SpatialMap = _context.SpatialMap, 
                TriangleSpatialMap = _context.TriangleSpatialMap, 
                
                Grid = _grid,
                DeltaTime = dt,
                BarrierRatio = Config.BarrierStiffnessRatio,
            };
            dependency = rbBarrier.Schedule(bodyCount, 16, dependency);

            // 5. Integration
            var rbInteg = new PhysicsJobs.RigidBodyVelocityUpdateJob
            {
                Velocity = _context.BodyVelocity.AsArray(),
                AngularVelocity = _context.BodyAngularVelocity.AsArray(),
                Position = _context.BodyPosition.AsArray(),
                Rotation = _context.BodyRotation.AsArray(),
                PredictedPos = _context.BodyPredictedPos.AsArray(),
                PredictedRot = _context.BodyPredictedRot.AsArray(),
                DeltaTime = dt
            };
            dependency = rbInteg.Schedule(bodyCount, 64, dependency);

            // 6. Visual Sync
            var vizJob = new PhysicsJobs.CopyParticlesJob
            {
                Output = _context.CurrentPosition.AsArray(),
                Input = _context.PredictedPosition.AsArray()
            };
            dependency = vizJob.Schedule(particleCount, 64, dependency);

            return dependency;
        }

        public void Dispose()
        {
            _context.Dispose();
        }
    }
}