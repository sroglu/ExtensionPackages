using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace mehmetsrl.Physics.XPBD.Core
{
    public static class PhysicsJobs
    {
        [BurstCompile]
        public struct RigidBodyPredictionJob : IJobParallelFor
        {
            public NativeArray<float3> PredictedPos;
            public NativeArray<quaternion> PredictedRot;
            public NativeArray<float3> Velocity;
            public NativeArray<float3> AngularVelocity;
            [ReadOnly] public NativeArray<float3> Position;
            [ReadOnly] public NativeArray<quaternion> Rotation;
            [ReadOnly] public NativeArray<float> InvMass;
            public float3 Gravity;
            public float DeltaTime;

            public void Execute(int i)
            {
                if (InvMass[i] == 0.0f)
                {
                    PredictedPos[i] = Position[i];
                    PredictedRot[i] = Rotation[i];
                    return;
                }

                Velocity[i] += Gravity * DeltaTime;
                PredictedPos[i] = Position[i] + Velocity[i] * DeltaTime;
                float3 omega = AngularVelocity[i];
                quaternion q = Rotation[i];
                quaternion spin = new quaternion(omega.x, omega.y, omega.z, 0);
                quaternion dq = math.mul(spin, q);
                PredictedRot[i] = math.normalize(new quaternion(q.value + dq.value * 0.5f * DeltaTime));
            }
        }

        [BurstCompile]
        public struct SyncProxiesJob : IJobParallelFor
        {
            [WriteOnly] public NativeArray<float3> PredictedParticlePos;
            [ReadOnly] public NativeArray<int> ParticleBodyID;
            [ReadOnly] public NativeArray<float3> LocalPos;
            [ReadOnly] public NativeArray<float3> BodyPos;
            [ReadOnly] public NativeArray<quaternion> BodyRot;

            public void Execute(int i)
            {
                int bodyIdx = ParticleBodyID[i];
                PredictedParticlePos[i] = BodyPos[bodyIdx] + math.rotate(BodyRot[bodyIdx], LocalPos[i]);
            }
        }

        [BurstCompile]
        public struct BuildSpatialMapJob : IJobParallelFor
        {
            [WriteOnly] public NativeParallelMultiHashMap<int, int>.ParallelWriter MapWriter;
            [ReadOnly] public NativeArray<float3> Positions;
            public SpatialGrid Grid;

            public void Execute(int index)
            {
                MapWriter.Add(Grid.GetHashFromPosition(Positions[index]), index);
            }
        }

        [BurstCompile]
        public struct BuildTriangleGridJob : IJobParallelFor
        {
            [WriteOnly] public NativeParallelMultiHashMap<int, int>.ParallelWriter MapWriter;
            [ReadOnly] public NativeArray<int3> Triangles;
            [ReadOnly] public NativeArray<float3> Positions;
            public SpatialGrid Grid;
            public float Margin;

            public void Execute(int i)
            {
                int3 t = Triangles[i];
                float3 p0 = Positions[t.x], p1 = Positions[t.y], p2 = Positions[t.z];
                float3 min = math.min(p0, math.min(p1, p2)) - Margin;
                float3 max = math.max(p0, math.max(p1, p2)) + Margin;
                int3 minCell = Grid.GetCellCoordinate(min), maxCell = Grid.GetCellCoordinate(max);
                for (int x = minCell.x; x <= maxCell.x; x++)
                for (int y = minCell.y; y <= maxCell.y; y++)
                for (int z = minCell.z; z <= maxCell.z; z++)
                    MapWriter.Add(Grid.GetHash(new int3(x, y, z)), i);
            }
        }

        [BurstCompile]
        public struct RigidBodyBarrierJob : IJobParallelFor
        {
            public NativeArray<float3> BodyPos;
            public NativeArray<quaternion> BodyRot;
            [ReadOnly] public NativeArray<int2> BodyParticleSlices;

            [ReadOnly] public NativeArray<float3> ParticlePos;
            [ReadOnly] public NativeArray<float> ParticleRadius;
            [ReadOnly] public NativeArray<int> ParticleBodyID;
            [ReadOnly] public NativeArray<float> BodyInvMass;
            [ReadOnly] public NativeArray<float3> BodyInvInertia;

            [ReadOnly] public NativeParallelMultiHashMap<int, int> SpatialMap;
            [ReadOnly] public NativeParallelMultiHashMap<int, int> TriangleSpatialMap;
            [ReadOnly] public NativeArray<int3> Triangles;
            [ReadOnly] public NativeArray<int> TriangleBodyIDs; // NEW

            public SpatialGrid Grid;
            public float DeltaTime;
            public float BarrierRatio;

            public void Execute(int bodyIdx)
            {
                if (BodyInvMass[bodyIdx] == 0.0f) return;

                int2 slice = BodyParticleSlices[bodyIdx];
                int start = slice.x;
                int count = slice.y;
                if (count == 0) return;

                for (int i = start; i < start + count; i++)
                {
                    float3 p_i = ParticlePos[i];
                    float r_i = ParticleRadius[i];
                    int3 cell = Grid.GetCellCoordinate(p_i);

                    for (int x = -1; x <= 1; x++)
                    {
                        for (int y = -1; y <= 1; y++)
                        {
                            for (int z = -1; z <= 1; z++)
                            {
                                int hash = Grid.GetHash(cell + new int3(x, y, z));

                                // A. TRIANGLES
                                if (TriangleSpatialMap.TryGetFirstValue(hash, out int triIdx, out var itTri))
                                {
                                    do
                                    {
                                        // IGNORE SELF TRIANGLES
                                        if (TriangleBodyIDs[triIdx] == bodyIdx) continue;

                                        int3 t = Triangles[triIdx];
                                        float3 p0 = ParticlePos[t.x], p1 = ParticlePos[t.y], p2 = ParticlePos[t.z];
                                        float3 v01 = p1 - p0, v02 = p2 - p0;
                                        float3 normal = math.normalize(math.cross(v01, v02));
                                        float distToPlane = math.dot(p_i - p0, normal);
                                        float barrierDist = r_i + 0.02f;

                                        if (math.abs(distToPlane) < barrierDist)
                                        {
                                            float3 proj = p_i - distToPlane * normal;
                                            if (IsPointInTriangle(proj, p0, p1, p2))
                                            {
                                                float C = distToPlane - barrierDist;
                                                if (C < 0) ApplyImpulse(bodyIdx, normal, C, p_i);
                                            }
                                        }
                                    } while (TriangleSpatialMap.TryGetNextValue(out triIdx, ref itTri));
                                }

                                // B. PARTICLES (Body vs Body)
                                if (SpatialMap.TryGetFirstValue(hash, out int j, out var itPart))
                                {
                                    do
                                    {
                                        if (i == j) continue;
                                        if (ParticleBodyID[j] == bodyIdx) continue;

                                        float3 p_j = ParticlePos[j];
                                        float r_j = ParticleRadius[j];
                                        float3 delta = p_i - p_j;
                                        float distSq = math.lengthsq(delta);
                                        float radSum = r_i + r_j;

                                        if (distSq < radSum * radSum)
                                        {
                                            float dist = math.sqrt(distSq);
                                            if (dist < 1e-9f)
                                            {
                                                delta = new float3(0, 1, 0);
                                                dist = 1e-9f;
                                            }

                                            float3 n = delta / dist;
                                            float C = dist - radSum;
                                            ApplyImpulse(bodyIdx, n, C, p_i);
                                        }
                                    } while (SpatialMap.TryGetNextValue(out j, ref itPart));
                                }
                            }
                        }
                    }
                }
            }

            private void ApplyImpulse(int bodyIdx, float3 n, float C, float3 contactPoint)
            {
                if (C >= 0) return;
                float3 com = BodyPos[bodyIdx];
                float3 r = contactPoint - com;

                float3 rxn = math.cross(r, n);
                quaternion rot = BodyRot[bodyIdx];
                float3 invInertia = BodyInvInertia[bodyIdx];

                float3 rotatedRxN = math.rotate(math.inverse(rot), rxn);
                float3 i_rxn = invInertia * rotatedRxN;
                float3 world_i_rxn = math.rotate(rot, i_rxn);

                float w_angular = math.dot(world_i_rxn, rxn);
                float w_total = BodyInvMass[bodyIdx] + w_angular;

                float lambda = -C / w_total;
                float3 impulse = n * lambda;

                BodyPos[bodyIdx] += impulse * BodyInvMass[bodyIdx];

                float3 dOmegaLocal = invInertia * math.rotate(math.inverse(rot), math.cross(r, impulse));
                float3 dOmegaWorld = math.rotate(rot, dOmegaLocal);

                quaternion spin = new quaternion(dOmegaWorld.x, dOmegaWorld.y, dOmegaWorld.z, 0);
                quaternion dq = math.mul(spin, rot);
                BodyRot[bodyIdx] = math.normalize(new quaternion(
                    BodyRot[bodyIdx].value + dq.value * 0.5f
                ));
            }

            private bool IsPointInTriangle(float3 p, float3 a, float3 b, float3 c)
            {
                float3 v0 = c - a, v1 = b - a, v2 = p - a;
                float d00 = math.dot(v0, v0), d01 = math.dot(v0, v1), d02 = math.dot(v0, v2);
                float d11 = math.dot(v1, v1), d12 = math.dot(v1, v2);
                float invDenom = 1.0f / (d00 * d11 - d01 * d01);
                float u = (d11 * d02 - d01 * d12) * invDenom;
                float v = (d00 * d12 - d01 * d02) * invDenom;
                return (u >= 0) && (v >= 0) && (u + v < 1);
            }
        }

        [BurstCompile]
        public struct RigidBodyVelocityUpdateJob : IJobParallelFor
        {
            public NativeArray<float3> Velocity;
            public NativeArray<float3> AngularVelocity;
            public NativeArray<float3> Position;
            public NativeArray<quaternion> Rotation;
            [ReadOnly] public NativeArray<float3> PredictedPos;
            [ReadOnly] public NativeArray<quaternion> PredictedRot;
            public float DeltaTime;

            public void Execute(int i)
            {
                float3 disp = PredictedPos[i] - Position[i];
                if (math.lengthsq(disp) > 1e-9f)
                    Velocity[i] = disp / DeltaTime;

                quaternion q_diff = math.mul(PredictedRot[i], math.inverse(Rotation[i]));
                if (q_diff.value.w < 0) q_diff.value = -q_diff.value;
                float3 axis = q_diff.value.xyz;
                AngularVelocity[i] = (axis * 2.0f) / DeltaTime;

                Position[i] = PredictedPos[i];
                Rotation[i] = PredictedRot[i];
            }
        }

        [BurstCompile]
        public struct CopyParticlesJob : IJobParallelFor
        {
            [WriteOnly] public NativeArray<float3> Output;
            [ReadOnly] public NativeArray<float3> Input;

            public void Execute(int i)
            {
                Output[i] = Input[i];
            }
        }
    }
}