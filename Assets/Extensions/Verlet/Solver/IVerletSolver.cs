using System.Collections.Generic;

namespace mehmetsrl.Extensions.VerletSystem.Solver
{
    public interface IVerletSolver
    {
        void SyncNodes(List<VerletNode> externalNodes, int layerMask); 
        void Solve(float dt);
        void RetrieveNodes(List<VerletNode> externalNodes);
    }
}