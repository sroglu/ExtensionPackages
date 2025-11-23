#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using mehmetsrl.Physics.XPBD.UnityIntegration;

namespace mehmetsrl.Physics.XPBD.Editor
{
    [CustomEditor(typeof(PhysicsSceneHook))]
    public class PhysicsSceneHookEditor : UnityEditor.Editor
    {
        SerializedProperty gravity;
        SerializedProperty solverIterations;
        SerializedProperty gridCellSize;
        SerializedProperty barrierStiffnessRatio;
        SerializedProperty maxParticles, maxBodies;

        void OnEnable()
        {
            gravity = serializedObject.FindProperty("Gravity");
            solverIterations = serializedObject.FindProperty("SolverIterations");
            gridCellSize = serializedObject.FindProperty("GridCellSize");

            barrierStiffnessRatio = serializedObject.FindProperty("BarrierStiffnessRatio");

            maxParticles = serializedObject.FindProperty("MaxParticles");
            maxBodies = serializedObject.FindProperty("MaxRigidBodies");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            // --- SIMULATION SETTINGS ---
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("Simulation Settings", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(gravity);
            EditorGUILayout.PropertyField(solverIterations);
            EditorGUILayout.PropertyField(gridCellSize);

            if (gridCellSize.floatValue < 0.1f)
                EditorGUILayout.HelpBox("Grid size too small! Performance may suffer.", MessageType.Warning);
            EditorGUILayout.EndVertical();

            // --- ELASTICITY (ANDO'S BARRIER) ---
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("Elasticity & Contact", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(barrierStiffnessRatio);
            EditorGUILayout.HelpBox(
                "Barrier Ratio: 1.0 = Barrier stiffness matches object. 10.0 = Barrier is 10x stiffer.",
                MessageType.Info);
            EditorGUILayout.EndVertical();

            // --- MEMORY CAPACITY ---
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("Memory Capacity (Startup Only)", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(maxParticles);
            EditorGUILayout.PropertyField(maxBodies);
            if (Application.isPlaying)
            {
                EditorGUILayout.HelpBox("Capacity changes require restart.", MessageType.Warning);
            }

            EditorGUILayout.EndVertical();

            serializedObject.ApplyModifiedProperties();
        }
    }
}
#endif