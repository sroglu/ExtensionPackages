using UnityEngine;
using Unity.Mathematics;
using mehmetsrl.Physics.XPBD.Core;

namespace mehmetsrl.Physics.XPBD.UnityIntegration
{
    // Component for a single particle/point mass (e.g. Falling Sphere).
    // Creates a minimal Rigid Body (one body, one collision proxy).
    public class XPBDBody : MonoBehaviour
    {
        [Header("Physical Properties")]
        public float Mass = 1.0f;
        public float Radius = 0.5f;
        public bool IsKinematic = false;

        [Header("Material")]
        [Tooltip("Inverse Stiffness. 0 = Hard, 1 = Soft.")]
        [Range(0f, 1f)]
        public float MaterialCompliance = 0.0f; 

        [Header("Constraints (Lock Axis)")]
        public bool LockX = false; public bool LockY = false; public bool LockZ = false;
        
        // Rotation locks are less relevant for a single point but kept for API consistency
        public bool LockRotationX = false; 
        public bool LockRotationY = false; 
        public bool LockRotationZ = false;

        // Internal State
        private int _bodyIndex = -1; // Index of the Rigid Body (COM)
        private int _proxyIndex = -1; // Index of the Collision Proxy
        private PhysicsSceneHook _hook;
        
        // --- HELPER: Inertia for a Sphere ---
        private float3 CalculateInverseInertia(float m, float r)
        {
            if (m < PhysicsConstants.Epsilon) return float3.zero; 
            // Solid sphere inertia tensor approximation
            float invInertiaVal = (2f / 5f) * (1f / (m * r * r));
            return new float3(invInertiaVal, invInertiaVal, invInertiaVal);
        }

        void Start()
        {
            _hook = FindFirstObjectByType<PhysicsSceneHook>();
            if (_hook == null)
            {
                Debug.LogError("[XPBDBody] Error: PhysicsSceneHook missing.");
                return;
            }
            RegisterBody();
        }

        void RegisterBody()
        {
            // 1. Calculate mass/inertia
            float simMass = IsKinematic ? 0.0f : Mass;
            float3 invInertia = CalculateInverseInertia(simMass, Radius);

            // 2. Apply Locks (Initial)
            float3 linLockMask = new float3(LockX ? 0f : 1f, LockY ? 0f : 1f, LockZ ? 0f : 1f);
            float3 angLockMask = new float3(LockRotationX ? 0f : 1f, LockRotationY ? 0f : 1f, LockRotationZ ? 0f : 1f);
            
            if (_hook.World == null) return;

            // 3. Add Rigid Body Entity (Center of Mass)
            _bodyIndex = _hook.World.AddBody(
                (float3)transform.position,
                (quaternion)transform.rotation,
                simMass * linLockMask.x, 
                invInertia * angLockMask,
                float3.zero 
            );
            
            // 4. Add Collision Proxy (Sensor)
            // Fix: Uses the correct 3-parameter signature and returns int index
            _proxyIndex = _hook.World.AddCollisionProxy(
                float3.zero, // Local offset is 0 for a single particle body
                Radius,
                _bodyIndex
            );
        }

        void FixedUpdate()
        {
            if (_bodyIndex == -1 || _hook == null || _hook.World == null) return;
            
            var simData = _hook.World.Context;

            // Safety check
            if (_bodyIndex >= simData.BodyCount) return;

            // 1. Runtime Parameter Sync
            float invMass = (Mass > 0.00001f) ? 1.0f / Mass : 0.0f;
            float3 invInertia = CalculateInverseInertia(Mass, Radius);
            
            float3 linLockMask = new float3(LockX ? 0f : 1f, LockY ? 0f : 1f, LockZ ? 0f : 1f);
            float3 angLockMask = new float3(LockRotationX ? 0f : 1f, LockRotationY ? 0f : 1f, LockRotationZ ? 0f : 1f);
            
            simData.BodyInverseMass[_bodyIndex] = IsKinematic ? 0.0f : invMass * linLockMask.x;
            simData.BodyInverseInertia[_bodyIndex] = invInertia * angLockMask;

            // 2. Position Sync
            if (IsKinematic)
            {
                // Unity -> Physics
                simData.BodyPosition[_bodyIndex] = (float3)transform.position;
                simData.BodyRotation[_bodyIndex] = (quaternion)transform.rotation;
                simData.BodyVelocity[_bodyIndex] = float3.zero; 
                simData.BodyAngularVelocity[_bodyIndex] = float3.zero;
            }
            else
            {
                // Physics -> Unity
                float3 pos = simData.BodyPosition[_bodyIndex];
                quaternion rot = simData.BodyRotation[_bodyIndex];
                
                if (!math.any(math.isnan(pos)))
                {
                    transform.position = (Vector3)pos;
                    transform.rotation = (Quaternion)rot;
                }
            }
        }
        
        void OnDrawGizmosSelected()
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(transform.position, Radius);
        }
    }
}