using System;
using Unity.Mathematics;

namespace mehmetsrl.Physics.PPF.Core
{
    // High level PPF engine skeleton. This file provides the core API and
    // the high-level step pipeline. Implementation details (jobs, Burst) will
    // be added in the UnityIntegration namespace later.
    public class PPFEngine : IDisposable
    {
        public PPFConfig Config;
        private PPFContext _context;
        private SpatialGrid _grid;

        public PPFContext Context => _context;

        public PPFEngine(PPFConfig? config = null, int maxProxies = 16000, int maxBodies = 1024)
        {
            Config = config ?? PPFConfig.Default;
            _context = new PPFContext(maxProxies, maxBodies);
            _grid = new SpatialGrid(Config.GridCellSize);
        }

        // --- API for building scenes ---
        public int AddBody(float3 position, quaternion rotation, float mass, float3 inverseInertia)
        {
            int idx = _context.BodyPosition.Length;
            _context.BodyPosition.Add(position);
            _context.BodyRotation.Add(rotation);
            _context.BodyVelocity.Add(float3.zero);
            _context.BodyAngularVelocity.Add(float3.zero);
            float invMass = mass > PPFConstants.Epsilon ? 1.0f / mass : 0.0f;
            _context.BodyInverseMass.Add(invMass);
            _context.BodyInverseInertia.Add(inverseInertia);
            _context.BodyProxySlices.Add(new int2(_context.Position.Length, 0));
            return idx;
        }

        // Proxies are the primitives we test for collisions. These may be particles on
        // rigid-body surfaces, or free-floating point proxies.
        public int AddProxy(float3 localOffset, float radius, int bodyIndex, float3 initialWorldPos)
        {
            int idx = _context.Position.Length;
            _context.Position.Add(initialWorldPos);
            _context.PredictedPosition.Add(initialWorldPos);
            _context.Velocity.Add(float3.zero);
            _context.Radius.Add(radius);
            _context.ProxyBodyID.Add(bodyIndex);
            _context.LocalOffset.Add(localOffset);

            // update slice
            int2 slice = _context.BodyProxySlices[bodyIndex];
            slice.y += 1;
            _context.BodyProxySlices[bodyIndex] = slice;

            return idx;
        }

        public void AddTriangle(int a, int b, int c, int bodyIndex)
        {
            _context.Triangles.Add(new int3(a, b, c));
            _context.TriangleBodyIDs.Add(bodyIndex);
        }

        // --- High-level step pipeline ---
        // This organizes the PPF algorithm: predict, broadphase, narrowphase (incl TOI), rollback/correct, final velocity solve.
        public void Step(float deltaTime)
        {
            if (_context.Position.Length == 0 && _context.BodyPosition.Length == 0) return;

            if (math.abs(_grid.CellSize - Config.GridCellSize) > 1e-5f)
                _grid = new SpatialGrid(Config.GridCellSize);

            // 1) Forward Prediction
            ForwardPrediction(deltaTime);

            // 2) Broadphase: build spatial maps
            BuildSpatialMaps();

            // 3) Narrowphase: contact generation + TOI discovery (swept tests)
            GenerateContactsAndTOI(deltaTime);

            // 4) If TOI events exist, perform conservative advancement / rollback passes
            if (_context.TOIEvents.Length > 0)
            {
                ProcessTOIEvents(deltaTime);
            }

            // 5) Backward Correction (PPF core): reposition contacts using proximal operator style correction
            BackwardCorrection(deltaTime);

            // 6) Velocity recovery and friction/resolution
            VelocitySolve(deltaTime);

            // 7) Clear transient buffers
            _context.ClearTransient();
        }

        // --- Pipeline stages (high-level placeholders) ---
        private void ForwardPrediction(float dt)
        {
            // Predict proxies and bodies forward: x_pred = x + v * dt + 0.5 * g * dt^2 (if desired)
            for (int i = 0; i < _context.Position.Length; i++)
            {
                float3 v = _context.Velocity[i];
                _context.PredictedPosition[i] = _context.Position[i] + v * dt;
            }

            for (int i = 0; i < _context.BodyPosition.Length; i++)
            {
                if (_context.BodyInverseMass[i] == 0.0f) continue; // kinematic
                _context.BodyPosition[i] += _context.BodyVelocity[i] * dt;
                // rotation integration omitted here; will be handled in UnityIntegration
            }
        }

        private void BuildSpatialMaps()
        {
            _context.SpatialMap.Clear();
            _context.TriangleSpatialMap.Clear();

            for (int i = 0; i < _context.PredictedPosition.Length; i++)
            {
                int hash = _grid.GetHashFromPosition(_context.PredictedPosition[i]);
                _context.SpatialMap.Add(hash, i);
            }

            for (int t = 0; t < _context.Triangles.Length; t++)
            {
                int3 tri = _context.Triangles[t];
                float3 p0 = _context.PredictedPosition[tri.x];
                float3 p1 = _context.PredictedPosition[tri.y];
                float3 p2 = _context.PredictedPosition[tri.z];

                float3 min = math.min(p0, math.min(p1, p2)) - PPFConstants.DefaultContactMargin;
                float3 max = math.max(p0, math.max(p1, p2)) + PPFConstants.DefaultContactMargin;
                int3 minCell = _grid.GetCellCoordinate(min);
                int3 maxCell = _grid.GetCellCoordinate(max);

                for (int x = minCell.x; x <= maxCell.x; x++)
                for (int y = minCell.y; y <= maxCell.y; y++)
                for (int z = minCell.z; z <= maxCell.z; z++)
                {
                    int hash = _grid.GetHash(new int3(x, y, z));
                    _context.TriangleSpatialMap.Add(hash, t);
                }
            }
        }

        private void GenerateContactsAndTOI(float dt)
        {
            // Clear previous contacts and TOI events
            _context.Contacts.Clear();
            _context.TOIEvents.Clear();

            // Iterate proxies and their neighbor cells to produce candidate contacts
            for (int i = 0; i < _context.PredictedPosition.Length; i++)
            {
                float3 pi0 = _context.Position[i]; // start
                float3 pi1 = _context.PredictedPosition[i]; // predicted end
                float3 vi = (pi1 - pi0) / math.max(dt, 1e-12f);
                float ri = _context.Radius[i];
                int3 cell = _grid.GetCellCoordinate(pi1);

                for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    int hash = _grid.GetHash(cell + new int3(dx, dy, dz));

                    // Particle neighbors (swept sphere-sphere TOI)
                    if (_context.SpatialMap.TryGetFirstValue(hash, out int j, out var it))
                    {
                        do
                        {
                            if (j == i) continue;
                            if (_context.ProxyBodyID[j] == _context.ProxyBodyID[i]) continue; // skip self-body

                            float3 pj0 = _context.Position[j];
                            float3 pj1 = _context.PredictedPosition[j];
                            float3 vj = (pj1 - pj0) / math.max(dt, 1e-12f);
                            float rj = _context.Radius[j];

                            float toi = SweptSphereSphereTOI(pi0, vi, pj0, vj, ri + rj);

                            if (toi <= 1.0f)
                            {
                                // If immediate overlap at t=0 also register contact
                                if (toi <= 0.0f) toi = 0.0f;

                                TOIEvent e = new TOIEvent
                                {
                                    ProxyIndex = i,
                                    OtherIndex = j,
                                    TOI = toi,
                                    Normal = math.normalize((pi0 + vi * toi * dt) - (pj0 + vj * toi * dt))
                                };
                                _context.TOIEvents.Add(e);

                                // Also record a simple contact for fallback/visualization
                                float3 posi = pi1;
                                float3 posj = pj1;
                                float3 d = posi - posj;
                                float distSq = math.lengthsq(d);
                                float rsum = ri + rj;
                                if (distSq < rsum * rsum)
                                {
                                    float dist = math.sqrt(distSq);
                                    float penetration = dist - rsum;
                                    float3 n = dist > PPFConstants.Epsilon ? d / dist : new float3(0, 1, 0);
                                    PPFContact c = new PPFContact
                                    {
                                        A = i, B = j, Type = ContactType.ParticleParticle, Normal = n,
                                        Penetration = penetration, TimeOfImpact = 1.0f
                                    };
                                    _context.Contacts.Add(c);
                                }
                            }
                        } while (_context.SpatialMap.TryGetNextValue(out j, ref it));
                    }

                    // Triangle candidates (swept sphere vs triangle via conservative advancement)
                    if (_context.TriangleSpatialMap.TryGetFirstValue(hash, out int triIdx, out var itTri))
                    {
                        do
                        {
                            int3 t = _context.Triangles[triIdx];
                            if (_context.TriangleBodyIDs[triIdx] == _context.ProxyBodyID[i]) continue; // skip self

                            float3 p0_0 = _context.Position[t.x];
                            float3 p1_0 = _context.Position[t.y];
                            float3 p2_0 = _context.Position[t.z];

                            float3 p0_1 = _context.PredictedPosition[t.x];
                            float3 p1_1 = _context.PredictedPosition[t.y];
                            float3 p2_1 = _context.PredictedPosition[t.z];

                            float3 vi_local = vi;
                            float3 v0 = (p0_1 - p0_0) / math.max(dt, 1e-12f);
                            float3 v1 = (p1_1 - p1_0) / math.max(dt, 1e-12f);
                            float3 v2 = (p2_1 - p2_0) / math.max(dt, 1e-12f);

                            float toi = SweptSphereTriangleTOI(pi0, vi_local, ri, p0_0, v0, p1_0, v1, p2_0, v2, dt);

                            if (toi <= 1.0f)
                            {
                                // compute normal at contact time
                                float3 spos = pi0 + vi_local * toi * dt;
                                float3 tp0 = p0_0 + v0 * toi * dt;
                                float3 tp1 = p1_0 + v1 * toi * dt;
                                float3 tp2 = p2_0 + v2 * toi * dt;

                                float3 v01 = tp1 - tp0;
                                float3 v02 = tp2 - tp0;
                                float3 normal = math.normalize(math.cross(v01, v02));

                                TOIEvent e = new TOIEvent
                                    { ProxyIndex = i, OtherIndex = triIdx, TOI = toi, Normal = normal };
                                _context.TOIEvents.Add(e);

                                // fallback contact at predicted time
                                float3 proj = spos - math.dot(spos - tp0, normal) * normal;
                                if (IsPointInTriangle(proj, tp0, tp1, tp2))
                                {
                                    float3 d = spos - proj;
                                    float penetration = math.length(d) - ri;
                                    PPFContact c = new PPFContact
                                    {
                                        A = i, B = triIdx, Type = ContactType.ParticleTriangle, Normal = normal,
                                        Penetration = penetration, TimeOfImpact = 1.0f
                                    };
                                    _context.Contacts.Add(c);
                                }
                            }
                        } while (_context.TriangleSpatialMap.TryGetNextValue(out triIdx, ref itTri));
                    }
                }
            }
        }

        // Analytic swept-sphere vs swept-sphere TOI in normalized time [0..1].
        // Solves |(p + v*t) - (q + u*t)|^2 = R^2, where t in [0,1] (normalized), v/u are velocities per unit time (i.e., per second),
        // but we treat t as fraction of the timestep here.
        private static float SweptSphereSphereTOI(float3 p, float3 v, float3 q, float3 u, float R)
        {
            // relative motion
            float3 s = p - q;
            float3 w = v - u; // relative velocity

            float a = math.dot(w, w);
            float b = 2.0f * math.dot(s, w);
            float c = math.dot(s, s) - R * R;

            if (a < PPFConstants.Epsilon)
            {
                // relative velocity is nearly zero -> no TOI unless already overlapping
                if (c <= 0.0f) return 0.0f; // currently overlapping
                return 1.0f + 1e-6f; // no collision within timestep
            }

            float disc = b * b - 4.0f * a * c;
            if (disc < 0.0f) return 1.0f + 1e-6f;

            float sqrtD = math.sqrt(disc);
            float t0 = (-b - sqrtD) / (2.0f * a);
            float t1 = (-b + sqrtD) / (2.0f * a);

            // we want the earliest t in [0,1]
            float toi = 1.0f + 1e-6f;
            if (t0 >= 0.0f && t0 <= 1.0f) toi = t0;
            else if (t1 >= 0.0f && t1 <= 1.0f) toi = t1;

            return toi;
        }

        // Conservative advancement for swept sphere vs triangle.
        // Returns TOI in normalized [0..1] or >1.0f if no impact.
        private static float SweptSphereTriangleTOI(float3 p0, float3 v, float r,
            float3 a0, float3 va,
            float3 b0, float3 vb,
            float3 c0, float3 vc,
            float dt)
        {
            // Conservative advancement: iteratively advance along time until collision or time exhausted.
            const int maxIter = 16;
            const float tol = 1e-4f;

            float t = 0.0f; // normalized
            for (int iter = 0; iter < maxIter && t <= 1.0f; iter++)
            {
                float3 ps = p0 + v * t * dt;
                float3 as_ = a0 + va * t * dt;
                float3 bs = b0 + vb * t * dt;
                float3 cs = c0 + vc * t * dt;

                // Signed distance from sphere center to triangle
                float3 closest;
                float signedDist = PointTriangleSignedDistance(ps, as_, bs, cs, out closest);

                float gap = signedDist - r; // positive if separated
                if (gap <= tol)
                {
                    return t; // contact at current t
                }

                // compute relative velocity along normal
                float3 triNormal = math.normalize(math.cross(bs - as_, cs - as_));
                float3 vRel = v - ((va + vb + vc) / 3.0f); // approximate triangle velocity
                float vn = math.dot(vRel, triNormal);
                if (vn >= -PPFConstants.Epsilon)
                {
                    // moving away or tangent -> no impact
                    return 1.0f + 1e-6f;
                }

                // conservative dt fraction estimate
                float step = gap / (-vn * dt);
                if (step <= 0.0f) step = 1e-4f;
                // advance time
                float nextT = t + step;
                if (nextT <= t + 1e-6f) nextT = t + 1e-6f; // ensure progress
                if (nextT > 1.0f) return 1.0f + 1e-6f;
                t = nextT;
            }

            return 1.0f + 1e-6f;
        }

        // Signed distance from point p to triangle (returns positive distance).
        // Also returns the closest point on triangle.
        private static float PointTriangleSignedDistance(float3 p, float3 a, float3 b, float3 c, out float3 closest)
        {
            // Compute closest point on triangle (standard algorithm)
            float3 ab = b - a;
            float3 ac = c - a;
            float3 ap = p - a;

            float d1 = math.dot(ab, ap);
            float d2 = math.dot(ac, ap);
            if (d1 <= 0.0f && d2 <= 0.0f)
            {
                closest = a;
                return math.length(p - a);
            }

            // Check vertex region B
            float3 bp = p - b;
            float d3 = math.dot(ab, bp);
            float d4 = math.dot(ac, bp);
            if (d3 >= 0.0f && d4 <= d3)
            {
                closest = b;
                return math.length(p - b);
            }

            // Edge region AB
            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
            {
                float v = d1 / (d1 - d3);
                closest = a + v * ab;
                return math.length(p - closest);
            }

            // Vertex region C
            float3 cp = p - c;
            float d5 = math.dot(ab, cp);
            float d6 = math.dot(ac, cp);
            if (d6 >= 0.0f && d5 <= d6)
            {
                closest = c;
                return math.length(p - c);
            }

            // Edge region AC
            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
            {
                float w = d2 / (d2 - d6);
                closest = a + w * ac;
                return math.length(p - closest);
            }

            // Edge region BC
            float va = d3 * d6 - d5 * d4;
            if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
            {
                float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                closest = b + w * (c - b);
                return math.length(p - closest);
            }

            // Inside face region. Compute projection onto plane.
            float3 normal = math.normalize(math.cross(ab, ac));
            float dist = math.dot(p - a, normal);
            closest = p - dist * normal;
            return math.abs(dist);
        }

        private void ProcessTOIEvents(float dt)
        {
            // Conservative advancement / rollback processing (simple event loop)
            if (_context.TOIEvents.Length == 0) return;

            // Sort TOIEvents ascending
            // Simple insertion sort for small counts
            for (int i = 1; i < _context.TOIEvents.Length; i++)
            {
                var key = _context.TOIEvents[i];
                int j = i - 1;
                while (j >= 0 && _context.TOIEvents[j].TOI > key.TOI)
                {
                    _context.TOIEvents[j + 1] = _context.TOIEvents[j];
                    j--;
                }

                _context.TOIEvents[j + 1] = key;
            }

            float remaining = 1.0f;
            float accumulated = 0.0f;

            // process events in order; for each event, advance to its TOI fraction (relative to original timestep)
            for (int ei = 0; ei < _context.TOIEvents.Length; ei++)
            {
                TOIEvent ev = _context.TOIEvents[ei];
                if (ev.TOI < accumulated - 1e-6f) continue; // already passed
                float localTOI = ev.TOI - accumulated;
                if (localTOI < 0.0f) localTOI = 0.0f;
                if (localTOI > remaining) break;

                // Advance predicted positions by localTOI*dt from their current positions
                float advanceT = localTOI * dt;
                for (int i = 0; i < _context.Position.Length; i++)
                {
                    float3 v = _context.Velocity[i];
                    _context.Position[i] = _context.Position[i]; // keep base
                    _context.PredictedPosition[i] = _context.Position[i] + v * advanceT;
                }

                // Apply a local positional correction at event
                if (ev.OtherIndex >= 0 && ev.OtherIndex < _context.Position.Length)
                {
                    // particle-particle
                    int a = ev.ProxyIndex;
                    int b = ev.OtherIndex;
                    float3 pa = _context.PredictedPosition[a];
                    float3 pb = _context.PredictedPosition[b];
                    float ra = _context.Radius[a];
                    float rb = _context.Radius[b];
                    float3 d = pa - pb;
                    float dist = math.length(d);
                    if (dist < 1e-6f) d = new float3(0, 1, 0);
                    dist = math.length(d);
                    float penetration = dist - (ra + rb);
                    if (penetration < 0.0f)
                    {
                        float invMa = _context.ProxyBodyID[a] >= 0
                            ? _context.BodyInverseMass[_context.ProxyBodyID[a]]
                            : 1.0f;
                        float invMb = _context.ProxyBodyID[b] >= 0
                            ? _context.BodyInverseMass[_context.ProxyBodyID[b]]
                            : 1.0f;
                        float w = invMa + invMb;
                        if (w > PPFConstants.Epsilon)
                        {
                            float3 n = d / dist;
                            float3 corr = n * (-penetration / w);
                            _context.PredictedPosition[a] += corr * invMa;
                            _context.PredictedPosition[b] -= corr * invMb;
                        }
                    }
                }
                else
                {
                    // particle-triangle
                    int proxy = ev.ProxyIndex;
                    int triIdx = ev.OtherIndex;
                    if (triIdx >= 0 && triIdx < _context.Triangles.Length)
                    {
                        int3 tri = _context.Triangles[triIdx];
                        float3 pa = _context.PredictedPosition[proxy];
                        float3 p0 = _context.PredictedPosition[tri.x];
                        float3 p1 = _context.PredictedPosition[tri.y];
                        float3 p2 = _context.PredictedPosition[tri.z];

                        float3 closest;
                        float sd = PointTriangleSignedDistance(pa, p0, p1, p2, out closest);
                        float penetration = sd - _context.Radius[proxy];
                        if (penetration < 0.0f)
                        {
                            float invMa = _context.ProxyBodyID[proxy] >= 0
                                ? _context.BodyInverseMass[_context.ProxyBodyID[proxy]]
                                : 1.0f;
                            float invTri = 1.0f / 3.0f; // distribute equally to tri verts
                            float3 n = math.normalize(pa - closest);
                            float3 corr = n * (-penetration / (invMa + invTri * 3.0f));

                            _context.PredictedPosition[proxy] += corr * invMa;
                            _context.PredictedPosition[tri.x] -= corr * invTri;
                            _context.PredictedPosition[tri.y] -= corr * invTri;
                            _context.PredictedPosition[tri.z] -= corr * invTri;
                        }
                    }
                }

                // advance accumulated time
                accumulated += localTOI;
                remaining = 1.0f - accumulated;
                if (remaining <= 1e-6f) break;

                // After processing the event, re-predict remaining positions for next events
                float remDt = remaining * dt;
                for (int i = 0; i < _context.Position.Length; i++)
                {
                    float3 v = _context.Velocity[i];
                    _context.PredictedPosition[i] = _context.Position[i] + v * (accumulated * dt) + v * remDt;
                }
            }
        }

        private void BackwardCorrection(float dt)
        {
            // The PPF rollback and proximal correction core.
            // A simple proximal-style correction: move predictive positions to eliminate penetration
            // Note: This is a simplified model; Zozo/PPF use more advanced proximal solvers.

            for (int i = 0; i < _context.Contacts.Length; i++)
            {
                PPFContact c = _context.Contacts[i];
                if (c.Penetration >= 0) continue; // no penetration

                if (c.Type == ContactType.ParticleParticle)
                {
                    int a = c.A;
                    int b = c.B;
                    float3 n = c.Normal;
                    float C = c.Penetration; // negative

                    float invMa = _context.ProxyBodyID[a] >= 0
                        ? _context.BodyInverseMass[_context.ProxyBodyID[a]]
                        : 1.0f;
                    float invMb = _context.ProxyBodyID[b] >= 0
                        ? _context.BodyInverseMass[_context.ProxyBodyID[b]]
                        : 1.0f;

                    float w = invMa + invMb;
                    if (w <= PPFConstants.Epsilon) continue;

                    float3 correction = n * (-C / w);
                    // distribute
                    _context.PredictedPosition[a] += correction * invMa;
                    _context.PredictedPosition[b] -= correction * invMb;
                }
                else if (c.Type == ContactType.ParticleTriangle)
                {
                    int proxy = c.A;
                    int triIdx = c.B;
                    float3 n = c.Normal;
                    float C = c.Penetration;

                    float invMa = _context.ProxyBodyID[proxy] >= 0
                        ? _context.BodyInverseMass[_context.ProxyBodyID[proxy]]
                        : 1.0f;

                    // Distribute correction to triangle vertices equally (simple approach)
                    int3 t = _context.Triangles[triIdx];
                    float3 corr = n * (-C / (invMa + 3.0f));

                    _context.PredictedPosition[proxy] += corr * invMa;
                    _context.PredictedPosition[t.x] -= corr * (1.0f / 3.0f);
                    _context.PredictedPosition[t.y] -= corr * (1.0f / 3.0f);
                    _context.PredictedPosition[t.z] -= corr * (1.0f / 3.0f);
                }
            }
        }

        private void VelocitySolve(float dt)
        {
            // Recover velocities from corrected predicted positions
            for (int i = 0; i < _context.Position.Length; i++)
            {
                float3 old = _context.Position[i];
                float3 pred = _context.PredictedPosition[i];
                _context.Velocity[i] = (pred - old) / dt;
                _context.Position[i] = pred;
            }

            // Bodies: recompute COM velocity from their proxy velocities (approximation)
            for (int b = 0; b < _context.BodyPosition.Length; b++)
            {
                int2 slice = _context.BodyProxySlices[b];
                if (slice.y == 0) continue;

                float3 avgVel = float3.zero;
                for (int pi = slice.x; pi < slice.x + slice.y; pi++)
                {
                    avgVel += _context.Velocity[pi];
                }

                avgVel /= (float)slice.y;
                _context.BodyVelocity[b] = avgVel;
                _context.BodyPosition[b] += avgVel * dt;
            }
        }

        private static bool IsPointInTriangle(float3 p, float3 a, float3 b, float3 c)
        {
            float3 v0 = c - a;
            float3 v1 = b - a;
            float3 v2 = p - a;
            float d00 = math.dot(v0, v0);
            float d01 = math.dot(v0, v1);
            float d02 = math.dot(v0, v2);
            float d11 = math.dot(v1, v1);
            float d12 = math.dot(v1, v2);
            float denom = d00 * d11 - d01 * d01;
            if (math.abs(denom) < PPFConstants.Epsilon) return false;
            float invDenom = 1.0f / denom;
            float u = (d11 * d02 - d01 * d12) * invDenom;
            float v = (d00 * d12 - d01 * d02) * invDenom;
            return (u >= 0) && (v >= 0) && (u + v <= 1);
        }

        public void Clear()
        {
            _context.ClearAll();
        }

        public void Dispose()
        {
            _context?.Dispose();
        }
    }
}