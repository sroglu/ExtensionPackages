using UnityEngine;
using System.Collections.Generic;
using mehmetsrl.Extensions.VerletSystem.Solver;

namespace mehmetsrl.Extensions.VerletSystem.Core
{
    public static class VerletSimulator
    {
        public static void Step(List<VerletNode> nodes, RopePhysicsSettings settings, float dt)
        {
            float dtSq = dt * dt;
            float dragFactor = 1.0f - settings.drag;

            for (int i = 0; i < nodes.Count; i++)
            {
                VerletNode node = nodes[i];
                if (node.isStatic) continue;

                // Verlet Integration: x(t+1) = x(t) + (x(t) - x(t-1)) + a * dt^2
                Vector3 velocity = (node.position - node.prevPosition) * dragFactor;
                node.prevPosition = node.position;
                node.position = node.position + velocity + (settings.gravity * dtSq);
                
                nodes[i] = node; // Write back to list (struct value type)
            }
        }

        public static void ApplyConstraints(List<VerletNode> nodes, float nominalLength, float stiffness, Vector3? startPos, Vector3? endPos)
        {
            // 1. Distance Constraints (Structural Integrity)
            for (int i = 0; i < nodes.Count - 1; i++)
            {
                VerletNode nA = nodes[i];
                VerletNode nB = nodes[i + 1];

                Vector3 delta = nB.position - nA.position;
                float dist = delta.magnitude;
                
                if (dist < 1e-5f) dist = 1e-5f; // Prevent division by zero

                float error = dist - nominalLength;
                float scalar = (error / dist) * 0.5f * stiffness;
                Vector3 correction = delta * scalar;

                if (!nA.isStatic) 
                {
                    nA.position += correction;
                    nodes[i] = nA;
                }
                if (!nB.isStatic) 
                {
                    nB.position -= correction;
                    nodes[i + 1] = nB;
                }
            }

            // 2. Anchor Constraints (Pinning)
            if (nodes.Count > 0)
            {
                if (startPos.HasValue)
                {
                    VerletNode head = nodes[0];
                    head.position = startPos.Value;
                    nodes[0] = head;
                }

                if (endPos.HasValue)
                {
                    VerletNode tail = nodes[^1];
                    tail.position = endPos.Value;
                    nodes[^1] = tail;
                }
            }
        }
    }
}