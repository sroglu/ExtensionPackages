using System.Collections.Generic;
using mehmetsrl.Extensions.VerletSystem.Solver;
using UnityEngine;

namespace mehmetsrl.Extensions.VerletSystem
{
    public interface IRopeVisualizer
    {
        void UpdateVisualization(List<VerletNode> nodes, float thickness);
        void SetColorSegments(List<RopeColorSection> segments);
        void SetVisible(bool isVisible);
        
        // YENİ: Dışarıdan materyal atanabilmesi için
        void SetMaterial(Material material); 
    }
}