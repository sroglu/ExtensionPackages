using UnityEngine;

// Namespace to encapsulate the PPF library logic
namespace mehmetsrl.Extensions.VerletSystem.Solver
{
    /// <summary>
    /// Represents a single Finite Element node in the simulation.
    /// This is the atomic unit of the PPF solver.
    /// </summary>
   
    public struct VerletNode
    {
        public Vector3 position;      // Current spatial position (t)
        public Vector3 prevPosition;  // Previous spatial position (t-dt)
        public Vector3 velocity;      // Explicit velocity (optional hybrid approach)
        public float mass;            // Inverse mass is often preferred, but we store mass for clarity
        public float radius;          // The collision radius (thickness)
        public int id;                // Unique identifier
        public bool isStatic;         // If true, infinite mass (pinned)
        
        // Material properties for this specific node
        public float friction;
        public float restitution;

        public VerletNode(int id, Vector3 pos, float mass, float radius)
        {
            this.id = id;
            this.position = pos;
            this.prevPosition = pos;
            this.velocity = Vector3.zero;
            this.mass = mass;
            this.radius = radius;
            this.isStatic = false;
            this.friction = 0.5f;
            this.restitution = 0.0f;
        }
    }
    
}