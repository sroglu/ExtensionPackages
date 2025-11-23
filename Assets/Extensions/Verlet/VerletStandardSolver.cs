using System;
using System.Collections.Generic;
using mehmetsrl.Extensions.VerletSystem.Solver;
using UnityEngine;

namespace mehmetsrl.Extensions.PPF.Solvers
{
    [Serializable]
    public class VerletStandardSolver : MonoBehaviour, IVerletSolver
    {
        [Header("Optimization")] [SerializeField]
        private int _maxColliderPerNode = 16;

        [SerializeField] private int _initialCapacity = 128;

        [Header("Global Solver Settings")] [SerializeField]
        private int _collisionIterations = 2;

        // Shared Buffers
        private VerletNode[] _sharedBuffer;
        private Collider[] _colliderBuffer;
        private static readonly List<VerletContact> _contacts = new(64);

        private int _currentCount = 0;
        private int _currentLayerMask = -1; // Stores the mask for the current rope

        private void Awake()
        {
            _sharedBuffer = new VerletNode[_initialCapacity];
            _colliderBuffer = new Collider[_maxColliderPerNode];
        }

        // Updated to accept LayerMask
        public void SyncNodes(List<VerletNode> externalNodes, int layerMask)
        {
            int count = externalNodes.Count;
            _currentCount = count;
            _currentLayerMask = layerMask; // Store the mask for this specific rope

            // Resize buffer if needed
            if (_sharedBuffer == null || _sharedBuffer.Length < count)
            {
                int newSize = Mathf.NextPowerOfTwo(count);
                _sharedBuffer = new VerletNode[newSize];
            }

            // Copy data
            for (int i = 0; i < count; i++)
            {
                _sharedBuffer[i] = externalNodes[i];
            }
        }

        public void Solve(float dt)
        {
            GenerateContacts();

            for (int k = 0; k < _collisionIterations; k++)
            {
                SolveContacts();
            }
        }

        public void RetrieveNodes(List<VerletNode> externalNodes)
        {
            for (int i = 0; i < _currentCount; i++)
            {
                externalNodes[i] = _sharedBuffer[i];
            }
        }

        // --- Internal Physics ---

        private void GenerateContacts()
        {
            _contacts.Clear();

            for (int i = 0; i < _currentCount; i++)
            {
                if (_sharedBuffer[i].isStatic) continue;

                // 
                // We pass _currentLayerMask here to filter collisions
                int hitCount = UnityEngine.Physics.OverlapSphereNonAlloc(
                    _sharedBuffer[i].position,
                    _sharedBuffer[i].radius,
                    _colliderBuffer,
                    _currentLayerMask // <--- USED HERE
                );

                for (int c = 0; c < hitCount; c++)
                {
                    Collider col = _colliderBuffer[c];

                    Vector3 closest = col.ClosestPoint(_sharedBuffer[i].position);
                    Vector3 vec = _sharedBuffer[i].position - closest;

                    float distSq = vec.sqrMagnitude;
                    float radSq = _sharedBuffer[i].radius * _sharedBuffer[i].radius;

                    if (distSq < radSq)
                    {
                        float dist = Mathf.Sqrt(distSq);
                        Vector3 normal = (dist < Mathf.Epsilon) ? Vector3.up : vec / dist;
                        float penetration = _sharedBuffer[i].radius - dist;

                        _contacts.Add(new VerletContact
                        {
                            nodeId = i,
                            point = closest,
                            normal = normal,
                            penetration = penetration
                        });
                    }
                }
            }
        }

        private void SolveContacts()
        {
            foreach (var contact in _contacts)
            {
                ref VerletNode node = ref _sharedBuffer[contact.nodeId];

                // 1. Projection
                node.position += contact.normal * contact.penetration;

                // 2. Friction
                Vector3 vel = node.position - node.prevPosition;
                Vector3 normalVel = Vector3.Project(vel, contact.normal);
                Vector3 tangentVel = vel - normalVel;

                tangentVel *= (1.0f - node.friction);

                node.prevPosition = node.position - (normalVel + tangentVel);
            }
        }
    }
}