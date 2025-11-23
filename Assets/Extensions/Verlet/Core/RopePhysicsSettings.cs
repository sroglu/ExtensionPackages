using UnityEngine;
using System;

namespace mehmetsrl.Extensions.VerletSystem.Core
{
    [Serializable]
    public class RopePhysicsSettings
    {
        [Header("Physical Properties")] public float massPerNode = 0.5f;
        public float thickness = 0.1f;
        public float drag = 0.01f;
        public Vector3 gravity = new Vector3(0, -9.81f, 0);
        public float friction = 0.4f;

        [Header("Constraints")] public int constraintIterations = 8;
        [Range(0.1f, 1f)] public float stiffness = 1.0f;

        [Header("Collision")] public LayerMask collisionLayers;
    }
}