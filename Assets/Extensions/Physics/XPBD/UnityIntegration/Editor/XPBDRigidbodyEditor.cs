#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

namespace mehmetsrl.Physics.XPBD.UnityIntegration.Editor
{
    [CustomEditor(typeof(XPBDRigidBody))]
    [CanEditMultipleObjects]
    public class XPBDRigidBodyEditor : UnityEditor.Editor
    {
        SerializedProperty totalMass, particleRadius, isKinematic, config;
        SerializedProperty generationMode, voxelDensity, maxVertexLimit; // Topology settings
        SerializedProperty lockX, lockY, lockZ, lockRotX, lockRotY, lockRotZ;

        void OnEnable()
        {
            // Physical
            totalMass = serializedObject.FindProperty("TotalMass");
            particleRadius = serializedObject.FindProperty("ParticleRadius");
            isKinematic = serializedObject.FindProperty("IsKinematic");
            config = serializedObject.FindProperty("Config");

            // Topology
            generationMode = serializedObject.FindProperty("GenerationMode");
            maxVertexLimit = serializedObject.FindProperty("MaxVertexLimit");

            // Constraints
            lockX = serializedObject.FindProperty("LockX");
            lockY = serializedObject.FindProperty("LockY");
            lockZ = serializedObject.FindProperty("LockZ");
            lockRotX = serializedObject.FindProperty("LockRotationX");
            lockRotY = serializedObject.FindProperty("LockRotationY");
            lockRotZ = serializedObject.FindProperty("LockRotationZ");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            // Reset label width to defaults to prevent bleeding
            EditorGUIUtility.labelWidth = 0;

            // --- PHYSICAL PROPERTIES ---
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("Physical Properties", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(totalMass);
            EditorGUILayout.PropertyField(particleRadius);
            EditorGUILayout.PropertyField(isKinematic);

            EditorGUILayout.Space(5);
            EditorGUILayout.PropertyField(config); // Deformation Compliance inside
            EditorGUILayout.EndVertical();

            // --- TOPOLOGY ---
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("Topology Generation", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(generationMode);

            // Conditional Logic: Show Vertex Limit only for MeshVertices
            if (generationMode.enumValueIndex == (int)BodyTopology.MeshVertices)
            {
                EditorGUI.indentLevel++;
                EditorGUILayout.PropertyField(maxVertexLimit);
                // Add a small info box if limit is high
                if (maxVertexLimit.intValue > 5000)
                    EditorGUILayout.HelpBox("High vertex limits may cause lag.", MessageType.Warning);
                EditorGUI.indentLevel--;
            }

            EditorGUILayout.EndVertical();

            // --- CONSTRAINTS (The Fix) ---
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("Constraints", EditorStyles.boldLabel);

            DrawConstraintRow("Freeze Position", lockX, lockY, lockZ);
            DrawConstraintRow("Freeze Rotation", lockRotX, lockRotY, lockRotZ);

            EditorGUILayout.EndVertical();

            serializedObject.ApplyModifiedProperties();
        }

        // Layout-Safe Drawing Method (Fixes "Cons" overlap)
        private void DrawConstraintRow(string label, SerializedProperty x, SerializedProperty y, SerializedProperty z)
        {
            Rect rect = EditorGUILayout.GetControlRect();

            // 1. Draw Label (Left Side)
            // Use standard label width from Unity to align with other fields
            Rect labelRect = new Rect(rect.x, rect.y, EditorGUIUtility.labelWidth, rect.height);
            EditorGUI.LabelField(labelRect, label);

            // 2. Calculate Start Position for Toggles
            float currentX = rect.x + EditorGUIUtility.labelWidth;

            // 3. Save State
            float oldLabelW = EditorGUIUtility.labelWidth;
            int oldIndent = EditorGUI.indentLevel;

            // 4. Configure for tight packing
            EditorGUI.indentLevel = 0;
            EditorGUIUtility.labelWidth = 12f; // Width of "X", "Y", "Z" letters

            float toggleWidth = 32f; // Standard width for checkbox + letter

            // Draw X
            Rect rectX = new Rect(currentX, rect.y, toggleWidth, rect.height);
            EditorGUI.PropertyField(rectX, x, new GUIContent("X"));

            // Draw Y
            Rect rectY = new Rect(currentX + toggleWidth, rect.y, toggleWidth, rect.height);
            EditorGUI.PropertyField(rectY, y, new GUIContent("Y"));

            // Draw Z
            Rect rectZ = new Rect(currentX + toggleWidth * 2, rect.y, toggleWidth, rect.height);
            EditorGUI.PropertyField(rectZ, z, new GUIContent("Z"));

            // 5. Restore State
            EditorGUIUtility.labelWidth = oldLabelW;
            EditorGUI.indentLevel = oldIndent;
        }
    }
}
#endif