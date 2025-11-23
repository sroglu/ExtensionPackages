using UnityEngine;

namespace mehmetsrl.Extensions.VerletSystem.Solver
{
    /// <summary>
    /// Describes a collision manifold between a node and the environment.
    /// </summary>
    public struct VerletContact
    {
        public int nodeId;
        public Vector3 point;        // Closest point on the collider surface
        public Vector3 normal;       // Surface normal at the contact point
        public float penetration;    // Depth of penetration (always positive in solver context)
    }
}