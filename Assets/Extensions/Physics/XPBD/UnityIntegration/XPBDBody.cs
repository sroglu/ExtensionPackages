using UnityEngine;
using Unity.Mathematics;
using mehmetsrl.Physics.XPBD.Core;

namespace mehmetsrl.Physics.XPBD.UnityIntegration
{
    // Component for a single particle/point mass.
    public class XPBDBody : MonoBehaviour
    {
        public float Mass = 1.0f;
        public float Radius = 0.5f;
        public bool IsKinematic = false;

        [Tooltip("Inverse Stiffness. 0 = Hard, 1 = Soft.")]
        [Range(0f, 1f)]
        public float MaterialCompliance = 0.0f; 
        
        public bool LockX = false; public bool LockY = false; public bool LockZ = false;
        
        public bool LockRotationX = false; 
        public bool LockRotationY = false; 
        public bool LockRotationZ = false;

        // Internal State
        [SerializeField] private int _bodyIndex = -1; 
        [SerializeField] private int _proxyIndex = -1; 
        
        private PhysicsSceneHook _hook;
        
        private float3 CalculateInverseInertia(float m, float r)
        {
            if (m < PhysicsConstants.Epsilon) return float3.zero; 
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
            float simMass = IsKinematic ? 0.0f : Mass;
            float3 invInertia = CalculateInverseInertia(simMass, Radius);

            float3 linLockMask = new float3(LockX ? 0f : 1f, LockY ? 0f : 1f, LockZ ? 0f : 1f);
            float3 angLockMask = new float3(LockRotationX ? 0f : 1f, LockRotationY ? 0f : 1f, LockRotationZ ? 0f : 1f);
            
            if (_hook.World == null) return;

            _bodyIndex = _hook.World.AddBody(
                (float3)transform.position,
                (quaternion)transform.rotation,
                simMass * linLockMask.x, 
                invInertia * angLockMask,
                float3.zero 
            );
            
            _proxyIndex = _hook.World.AddCollisionProxy(
                float3.zero, 
                Radius,
                _bodyIndex
            );
        }

        void FixedUpdate()
        {
            if (_bodyIndex == -1 || _hook == null || _hook.World == null) return;
            
            var simData = _hook.World.Context;
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
                simData.BodyPosition[_bodyIndex] = (float3)transform.position;
                simData.BodyRotation[_bodyIndex] = (quaternion)transform.rotation;
                simData.BodyVelocity[_bodyIndex] = float3.zero; 
                simData.BodyAngularVelocity[_bodyIndex] = float3.zero;
            }
            else
            {
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