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

        public PhysicsEngine(int maxParticles = 10000)
        {
            Config = PhysicsConfig.Default;
            _context = new PhysicsContext(maxParticles);
            _grid = new SpatialGrid(Config.GridCellSize);
        }

        public PhysicsEngine(PhysicsConfig config, int maxParticles = 10000)
        {
            Config = config;
            _context = new PhysicsContext(maxParticles);
            _grid = new SpatialGrid(Config.GridCellSize);
        }

        public int AddBody(float3 position, float mass, float radius, float3 lockMask, int bodyID,
            float initialStiffness = 0.0f)
        {
            int index = _context.CurrentPosition.Length;
            _context.CurrentPosition.Add(position);
            _context.PredictedPosition.Add(position);
            _context.PreviousPosition.Add(position);
            _context.Velocity.Add(float3.zero);
            float invMass = mass > PhysicsConstants.Epsilon ? 1.0f / mass : 0.0f;
            _context.InverseMass.Add(invMass);
            _context.Radius.Add(radius);
            _context.BodyIDs.Add(bodyID);
            _context.PositionLockMask.Add(lockMask);
            _context.CalculatedLocalStiffness.Add(initialStiffness);
            return index;
        }

        public void AddConstraint(int indexA, int indexB, float restLength, float compliance)
        {
            _context.DistanceConstraints.Add(new int2(indexA, indexB));
            _context.RestLengths.Add(restLength);
            _context.Compliances.Add(compliance);
            float stiffness = (compliance < PhysicsConstants.Epsilon)
                ? PhysicsConstants.MaxStiffness
                : (1.0f / compliance);
            if (indexA < _context.CalculatedLocalStiffness.Length)
                _context.CalculatedLocalStiffness[indexA] += stiffness;
            if (indexB < _context.CalculatedLocalStiffness.Length)
                _context.CalculatedLocalStiffness[indexB] += stiffness;
        }

        // NEW: Add Triangles
        public void AddTriangle(int a, int b, int c)
        {
            _context.Triangles.Add(new int3(a, b, c));
        }

        public void Clear()
        {
            _context.Clear();
        }

        public void Step(float deltaTime)
        {
            int particleCount = _context.CurrentPosition.Length;
            if (particleCount == 0) return;

            float dt = deltaTime / Config.SolverIterations;
            if (math.abs(_grid.CellSize - Config.GridCellSize) > 1e-4f) _grid = new SpatialGrid(Config.GridCellSize);

            JobHandle handle = default;
            for (int i = 0; i < Config.SolverIterations; i++)
            {
                handle = RunSubStep(dt, handle, particleCount);
            }

            handle.Complete();
        }

        private JobHandle RunSubStep(float dt, JobHandle dependency, int count)
        {
            // 1. Prediction
            var predJob = new PhysicsJobs.PredictionJob
            {
                CurrentPosition = _context.CurrentPosition,
                PredictedPosition = _context.PredictedPosition,
                PreviousPosition = _context.PreviousPosition,
                Velocity = _context.Velocity,
                InverseMass = _context.InverseMass,
                PositionLockMask = _context.PositionLockMask,
                Gravity = Config.Gravity,
                DeltaTime = dt
            };
            dependency = predJob.Schedule(count, 64, dependency);

            // 2. Clear Maps
            dependency.Complete();
            _context.SpatialMap.Clear();
            _context.TriangleSpatialMap.Clear();

            // 3. Build Maps (Parallel)
            var buildPartMap = new PhysicsJobs.BuildSpatialMapJob
            {
                MapWriter = _context.SpatialMap.AsParallelWriter(),
                Positions = _context.PredictedPosition,
                Grid = _grid
            };
            dependency = buildPartMap.Schedule(count, 64, default);

            var buildTriMap = new PhysicsJobs.BuildTriangleGridJob
            {
                MapWriter = _context.TriangleSpatialMap.AsParallelWriter(),
                Triangles = _context.Triangles,
                Positions = _context.PredictedPosition,
                Grid = _grid,
                Margin = 0.2f
            };
            // Schedule triangle map in parallel with particle logic if possible, but sequential here for safety
            dependency = buildTriMap.Schedule(_context.Triangles.Length, 64, dependency);

            // 4. Solve Barriers

            // A. Particle-Particle (Ropes/Cloths)
            var particleBarrier = new PhysicsJobs.ParticleBarrierJob
            {
                PredictedPosition = _context.PredictedPosition,
                InverseMass = _context.InverseMass,
                Radius = _context.Radius,
                SpatialMap = _context.SpatialMap,
                BodyIDs = _context.BodyIDs,
                CalculatedLocalStiffness = _context.CalculatedLocalStiffness,
                Grid = _grid,
                DeltaTime = dt,
                BarrierStiffnessRatio = Config.BarrierStiffnessRatio
            };
            dependency = particleBarrier.Schedule(count, 16, dependency);

            // B. Point-Triangle (Rigid Bodies / Walls)
            var triBarrier = new PhysicsJobs.TriangleBarrierJob
            {
                PredictedPosition = _context.PredictedPosition,
                InverseMass = _context.InverseMass,
                Radius = _context.Radius,
                BodyIDs = _context.BodyIDs,
                Triangles = _context.Triangles,
                TriangleSpatialMap = _context.TriangleSpatialMap,
                CalculatedLocalStiffness = _context.CalculatedLocalStiffness,
                Grid = _grid,
                DeltaTime = dt,
                BarrierStiffnessRatio = Config.BarrierStiffnessRatio
            };
            dependency = triBarrier.Schedule(count, 16, dependency);

            // 5. Solve Topology
            var distJob = new PhysicsJobs.DistanceConstraintJob
            {
                PredictedPosition = _context.PredictedPosition,
                InverseMass = _context.InverseMass,
                Pairs = _context.DistanceConstraints,
                RestLengths = _context.RestLengths,
                Compliances = _context.Compliances,
                DeltaTime = dt
            };
            dependency = distJob.Schedule(dependency);

            // 6. Integrate
            var velJob = new PhysicsJobs.VelocityUpdateJob
            {
                Velocity = _context.Velocity,
                CurrentPosition = _context.CurrentPosition,
                PredictedPosition = _context.PredictedPosition,
                PreviousPosition = _context.PreviousPosition,
                DeltaTime = dt
            };
            dependency = velJob.Schedule(count, 64, dependency);

            return dependency;
        }

        public void Dispose()
        {
            _context.Dispose();
        }
    }
}