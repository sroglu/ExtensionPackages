using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace mehmetsrl.Physics.XPBD.Core
{
    public static class PhysicsJobs
    {
        // --------------------------------------------------------------------
        // 1. PREDICTION
        // --------------------------------------------------------------------
        [BurstCompile]
        public struct PredictionJob : IJobParallelFor
        {
            public NativeArray<float3> PredictedPosition;
            public NativeArray<float3> PreviousPosition;
            public NativeArray<float3> Velocity;
            [ReadOnly] public NativeArray<float3> CurrentPosition;
            [ReadOnly] public NativeArray<float> InverseMass;
            [ReadOnly] public NativeArray<float3> PositionLockMask;
            public float3 Gravity;
            public float DeltaTime;

            public void Execute(int i)
            {
                PreviousPosition[i] = CurrentPosition[i];
                if (InverseMass[i] == 0.0f)
                {
                    PredictedPosition[i] = CurrentPosition[i];
                    Velocity[i] = float3.zero;
                    return;
                }

                float3 v = Velocity[i] + (Gravity * DeltaTime);
                float3 displacement = v * DeltaTime;
                displacement *= PositionLockMask[i];
                PredictedPosition[i] = CurrentPosition[i] + displacement;
            }
        }

        // --------------------------------------------------------------------
        // 2. BROADPHASE (PARTICLES)
        // --------------------------------------------------------------------
        [BurstCompile]
        public struct BuildSpatialMapJob : IJobParallelFor
        {
            [WriteOnly] public NativeParallelMultiHashMap<int, int>.ParallelWriter MapWriter;
            [ReadOnly] public NativeArray<float3> Positions;
            public SpatialGrid Grid;

            public void Execute(int index)
            {
                int hash = Grid.GetHashFromPosition(Positions[index]);
                MapWriter.Add(hash, index);
            }
        }

        // --------------------------------------------------------------------
        // 3. BROADPHASE (TRIANGLES)
        // --------------------------------------------------------------------
        [BurstCompile]
        public struct BuildTriangleGridJob : IJobParallelFor
        {
            [WriteOnly] public NativeParallelMultiHashMap<int, int>.ParallelWriter MapWriter;
            [ReadOnly] public NativeArray<int3> Triangles;
            [ReadOnly] public NativeArray<float3> Positions;
            public SpatialGrid Grid;
            public float Margin; // Expand bounds to catch particles nearby

            public void Execute(int i)
            {
                int3 t = Triangles[i];
                float3 p0 = Positions[t.x];
                float3 p1 = Positions[t.y];
                float3 p2 = Positions[t.z];

                float3 min = math.min(p0, math.min(p1, p2)) - Margin;
                float3 max = math.max(p0, math.max(p1, p2)) + Margin;

                int3 minCell = Grid.GetCellCoordinate(min);
                int3 maxCell = Grid.GetCellCoordinate(max);

                for (int x = minCell.x; x <= maxCell.x; x++)
                {
                    for (int y = minCell.y; y <= maxCell.y; y++)
                    {
                        for (int z = minCell.z; z <= maxCell.z; z++)
                        {
                            int hash = Grid.GetHash(new int3(x, y, z));
                            MapWriter.Add(hash, i);
                        }
                    }
                }
            }
        }

        // --------------------------------------------------------------------
        // 4. SOLVER: PARTICLE BARRIER (Sphere-Sphere)
        // --------------------------------------------------------------------
        [BurstCompile]
        public struct ParticleBarrierJob : IJobParallelFor
        {
            [NativeDisableParallelForRestriction] public NativeArray<float3> PredictedPosition;
            [ReadOnly] public NativeArray<float> InverseMass;
            [ReadOnly] public NativeArray<float> Radius;
            [ReadOnly] public NativeParallelMultiHashMap<int, int> SpatialMap;
            [ReadOnly] public NativeArray<int> BodyIDs;
            [ReadOnly] public NativeArray<float> CalculatedLocalStiffness;
            public SpatialGrid Grid;
            public float DeltaTime;
            public float BarrierStiffnessRatio;

            public void Execute(int i)
            {
                if (InverseMass[i] == 0.0f) return;

                float3 x_i = PredictedPosition[i];
                float r_i = Radius[i];
                int bodyID_i = BodyIDs[i];
                int3 centerCell = Grid.GetCellCoordinate(x_i);

                float localStiffness = CalculatedLocalStiffness[i];
                if (localStiffness < 1e-5f) localStiffness = 1000.0f;
                float targetStiffness = localStiffness * BarrierStiffnessRatio;
                float baseCompliance = 1.0f / targetStiffness;

                for (int x = -1; x <= 1; x++)
                {
                    for (int y = -1; y <= 1; y++)
                    {
                        for (int z = -1; z <= 1; z++)
                        {
                            int hash = Grid.GetHash(centerCell + new int3(x, y, z));
                            if (SpatialMap.TryGetFirstValue(hash, out int j, out var it))
                            {
                                do
                                {
                                    if (i == j) continue;
                                    if (bodyID_i == BodyIDs[j]) continue;

                                    float3 x_j = PredictedPosition[j];
                                    float r_j = Radius[j];
                                    float3 delta = x_i - x_j;
                                    float distSq = math.lengthsq(delta);
                                    float combinedRadius = r_i + r_j;

                                    if (distSq < combinedRadius * combinedRadius)
                                    {
                                        float dist = math.sqrt(distSq);
                                        if (dist < PhysicsConstants.Epsilon)
                                        {
                                            delta = new float3(0, 1, 0);
                                            dist = PhysicsConstants.Epsilon;
                                        }

                                        float3 n = delta / dist;
                                        float C = dist - combinedRadius;

                                        float t = dist / combinedRadius;
                                        float cubic = t * t * t;
                                        float finalCompliance = baseCompliance * cubic;
                                        float alphaTilde = finalCompliance / (DeltaTime * DeltaTime);

                                        float wSum = InverseMass[i] + InverseMass[j];
                                        float denom = wSum + alphaTilde;

                                        if (denom > PhysicsConstants.Epsilon)
                                        {
                                            float dL = -C / denom;
                                            float3 corr = n * dL;
                                            PredictedPosition[i] += corr * (InverseMass[i] / wSum);
                                            x_i += corr * (InverseMass[i] / wSum);
                                        }
                                    }
                                } while (SpatialMap.TryGetNextValue(out j, ref it));
                            }
                        }
                    }
                }
            }
        }

        // --------------------------------------------------------------------
        // 5. SOLVER: TRIANGLE BARRIER (Point-Triangle)
        // --------------------------------------------------------------------
        [BurstCompile]
        public struct TriangleBarrierJob : IJobParallelFor
        {
            [NativeDisableParallelForRestriction] public NativeArray<float3> PredictedPosition;
            [ReadOnly] public NativeArray<float> InverseMass;
            [ReadOnly] public NativeArray<float> Radius;
            [ReadOnly] public NativeArray<int> BodyIDs;
            [ReadOnly] public NativeArray<int3> Triangles;
            [ReadOnly] public NativeParallelMultiHashMap<int, int> TriangleSpatialMap;
            [ReadOnly] public NativeArray<float> CalculatedLocalStiffness;

            public SpatialGrid Grid;
            public float DeltaTime;
            public float BarrierStiffnessRatio;

            public void Execute(int i)
            {
                float w_i = InverseMass[i];
                if (w_i == 0.0f) return;

                float3 p = PredictedPosition[i];
                float r = Radius[i];
                int myBodyID = BodyIDs[i];
                int3 cell = Grid.GetCellCoordinate(p);
                int hash = Grid.GetHash(cell);

                if (TriangleSpatialMap.TryGetFirstValue(hash, out int triIdx, out var it))
                {
                    do
                    {
                        int3 t = Triangles[triIdx];
                        if (BodyIDs[t.x] == myBodyID) continue;

                        float3 p0 = PredictedPosition[t.x];
                        float3 p1 = PredictedPosition[t.y];
                        float3 p2 = PredictedPosition[t.z];

                        // Triangle Normal & Barycentric Setup
                        float3 v01 = p1 - p0;
                        float3 v02 = p2 - p0;
                        float3 normal = math.cross(v01, v02);
                        float lenSq = math.lengthsq(normal);
                        if (lenSq < 1e-9f) continue;
                        normal *= (1.0f / math.sqrt(lenSq));

                        float distToPlane = math.dot(p - p0, normal);
                        float barrierDist = r + 0.05f; // Slightly thicker barrier for stability

                        if (math.abs(distToPlane) > barrierDist) continue;

                        float3 proj = p - distToPlane * normal;

                        // Barycentric Coordinates (u,v,w) for distribution
                        float3 bary = GetBarycentricCoordinates(proj, p0, p1, p2);

                        // Check if inside triangle (allowing small margin)
                        if (bary.x >= -0.01f && bary.y >= -0.01f && bary.z >= -0.01f)
                        {
                            float dist = math.abs(distToPlane);
                            float C = dist - barrierDist;

                            if (C < 0)
                            {
                                float3 n = normal * math.sign(distToPlane);

                                float k_local = CalculatedLocalStiffness[i];
                                if (k_local < 1e-5f) k_local = 1000.0f;
                                float compliance = 1.0f / (k_local * BarrierStiffnessRatio);

                                float t_ratio = dist / barrierDist;
                                float cubic = t_ratio * t_ratio * t_ratio;
                                float finalAlpha = compliance * cubic;
                                float alphaTilde = finalAlpha / (DeltaTime * DeltaTime);

                                // --- TWO-WAY MASS CALCULATION ---
                                // Triangle effective mass at contact point
                                float w0 = InverseMass[t.x];
                                float w1 = InverseMass[t.y];
                                float w2 = InverseMass[t.z];

                                // Distributed inverse mass (Approximate)
                                float w_tri = w0 * bary.x * bary.x + w1 * bary.y * bary.y + w2 * bary.z * bary.z;
                                float wSum = w_i + w_tri + alphaTilde;

                                if (wSum > 1e-9f)
                                {
                                    float deltaLambda = -C / wSum;
                                    float3 impulse = n * deltaLambda;

                                    // 1. Push Point
                                    PredictedPosition[i] += impulse * w_i;

                                    // 2. Push Triangle Vertices (Opposite direction)
                                    // Distribute force based on barycentric weights
                                    // Note: Writing to random indices is technically a race condition,
                                    // but for XPBD RigidBodies, the ShapeMatching step usually corrects the jitter.
                                    // For perfect safety, we would need atomic adds or coloring.

                                    if (w0 > 0) PredictedPosition[t.x] -= impulse * (w0 * bary.x);
                                    if (w1 > 0) PredictedPosition[t.y] -= impulse * (w1 * bary.y);
                                    if (w2 > 0) PredictedPosition[t.z] -= impulse * (w2 * bary.z);
                                }
                            }
                        }
                    } while (TriangleSpatialMap.TryGetNextValue(out triIdx, ref it));
                }
            }

            private bool IsPointInTriangle(float3 p, float3 a, float3 b, float3 c)
            {
                float3 v0 = c - a;
                float3 v1 = b - a;
                float3 v2 = p - a;
                float dot00 = math.dot(v0, v0);
                float dot01 = math.dot(v0, v1);
                float dot02 = math.dot(v0, v2);
                float dot11 = math.dot(v1, v1);
                float dot12 = math.dot(v1, v2);
                float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
                float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
                float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
                return (u >= 0) && (v >= 0) && (u + v < 1);
            }

            // Barycentric Helper
            private float3 GetBarycentricCoordinates(float3 p, float3 a, float3 b, float3 c)
            {
                float3 v0 = b - a, v1 = c - a, v2 = p - a;
                float d00 = math.dot(v0, v0);
                float d01 = math.dot(v0, v1);
                float d11 = math.dot(v1, v1);
                float d20 = math.dot(v2, v0);
                float d21 = math.dot(v2, v1);
                float denom = d00 * d11 - d01 * d01;

                if (math.abs(denom) < 1e-9f) return new float3(-1, -1, -1);

                float v = (d11 * d20 - d01 * d21) / denom;
                float w = (d00 * d21 - d01 * d20) / denom;
                float u = 1.0f - v - w;
                return new float3(u, v, w);
            }
        }

        // --------------------------------------------------------------------
        // 6. SOLVER: DISTANCE CONSTRAINTS
        // --------------------------------------------------------------------
        [BurstCompile]
        public struct DistanceConstraintJob : IJob
        {
            [NativeDisableParallelForRestriction] public NativeArray<float3> PredictedPosition;
            [ReadOnly] public NativeArray<float> InverseMass;
            [ReadOnly] public NativeArray<int2> Pairs;
            [ReadOnly] public NativeArray<float> RestLengths;
            [ReadOnly] public NativeArray<float> Compliances;
            public float DeltaTime;

            public void Execute()
            {
                float dt2 = DeltaTime * DeltaTime;
                for (int i = 0; i < Pairs.Length; i++)
                {
                    int a = Pairs[i].x;
                    int b = Pairs[i].y;
                    float wA = InverseMass[a];
                    float wB = InverseMass[b];
                    float wSum = wA + wB;
                    if (wSum == 0.0f) continue;

                    float3 posA = PredictedPosition[a];
                    float3 posB = PredictedPosition[b];
                    float3 delta = posA - posB;
                    float len = math.length(delta);
                    if (len < PhysicsConstants.Epsilon) continue;

                    float3 n = delta / len;
                    float C = len - RestLengths[i];
                    float alphaTilde = Compliances[i] / dt2;
                    float dL = -C / (wSum + alphaTilde);
                    float3 corr = n * dL;

                    if (wA > 0.0f) PredictedPosition[a] += corr * wA;
                    if (wB > 0.0f) PredictedPosition[b] -= corr * wB;
                }
            }
        }

        // --------------------------------------------------------------------
        // 7. INTEGRATION
        // --------------------------------------------------------------------
        [BurstCompile]
        public struct VelocityUpdateJob : IJobParallelFor
        {
            public NativeArray<float3> Velocity;
            public NativeArray<float3> CurrentPosition;
            [ReadOnly] public NativeArray<float3> PredictedPosition;
            [ReadOnly] public NativeArray<float3> PreviousPosition;
            public float DeltaTime;

            public void Execute(int i)
            {
                if (math.distance(PredictedPosition[i], CurrentPosition[i]) > PhysicsConstants.Epsilon)
                {
                    Velocity[i] = (PredictedPosition[i] - PreviousPosition[i]) / DeltaTime;
                }

                CurrentPosition[i] = PredictedPosition[i];
            }
        }
    }
}