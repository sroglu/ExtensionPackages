using UnityEngine;
using mehmetsrl.Physics.XPBD.Core;

namespace mehmetsrl.Physics.XPBD.UnityIntegration
{
    public class XPBDParticleRenderer : MonoBehaviour
    {
        public PhysicsSceneHook PhysicsScene;
        public Mesh SphereMesh;
        public Material SphereMaterial;
        public float VisualScale = 1.0f; 

        private ComputeBuffer _positionBuffer;
        private ComputeBuffer _radiusBuffer;
        private ComputeBuffer _argsBuffer;
        private uint[] _args = new uint[5];

        void LateUpdate()
        {
            if (PhysicsScene == null || PhysicsScene.World == null) return;
            PhysicsScene.CompleteJobs();

            var simData = PhysicsScene.World.Context;
            int count = simData.ParticleCount;
            if (count == 0) return;

            // 1. Buffers
            if (_positionBuffer == null || _positionBuffer.count != count)
            {
                if(_positionBuffer != null) _positionBuffer.Release();
                _positionBuffer = new ComputeBuffer(count, 12); 

                if(_radiusBuffer != null) _radiusBuffer.Release();
                _radiusBuffer = new ComputeBuffer(count, 4); 

                if(_argsBuffer != null) _argsBuffer.Release();
                _argsBuffer = new ComputeBuffer(1, _args.Length * sizeof(uint), ComputeBufferType.IndirectArguments);
            }

            // 2. Upload Data
            _positionBuffer.SetData(simData.CurrentPosition.AsArray());
            _radiusBuffer.SetData(simData.Radius.AsArray());

            // 3. Material Props
            SphereMaterial.SetBuffer("_Positions", _positionBuffer);
            SphereMaterial.SetBuffer("_Radii", _radiusBuffer);
            SphereMaterial.SetFloat("_Scale", VisualScale);

            // 4. Draw
            _args[0] = (uint)SphereMesh.GetIndexCount(0);
            _args[1] = (uint)count;
            _args[2] = (uint)SphereMesh.GetIndexStart(0);
            _args[3] = (uint)SphereMesh.GetBaseVertex(0);
            _args[4] = 0;
            _argsBuffer.SetData(_args);

            Bounds b = new Bounds(Vector3.zero, Vector3.one * 1000f);
            Graphics.DrawMeshInstancedIndirect(SphereMesh, 0, SphereMaterial, b, _argsBuffer);
        }

        void OnDisable()
        {
            _positionBuffer?.Release();
            _radiusBuffer?.Release();
            _argsBuffer?.Release();
        }
    }
}