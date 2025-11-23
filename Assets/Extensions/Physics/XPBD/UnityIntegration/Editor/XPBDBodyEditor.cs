#if UNITY_EDITOR
using mehmetsrl.Physics.XPBD.UnityIntegration;
using UnityEngine;
using UnityEditor;

namespace mehmetsrl.Physics.XPBD.Editor
{
    [CustomEditor(typeof(XPBDBody))]
    [CanEditMultipleObjects]
    public class XPBDBodyEditor : UnityEditor.Editor
    {
        SerializedProperty mass, radius, isKinematic;
        SerializedProperty materialCompliance;
        SerializedProperty lockX, lockY, lockZ;
        SerializedProperty lockRotX, lockRotY, lockRotZ;
        SerializedProperty bodyIndex, proxyIndex;

        void OnEnable()
        {
            mass = serializedObject.FindProperty("Mass");
            radius = serializedObject.FindProperty("Radius");
            isKinematic = serializedObject.FindProperty("IsKinematic");

            materialCompliance = serializedObject.FindProperty("MaterialCompliance");

            lockX = serializedObject.FindProperty("LockX");
            lockY = serializedObject.FindProperty("LockY");
            lockZ = serializedObject.FindProperty("LockZ");
            lockRotX = serializedObject.FindProperty("LockRotationX");
            lockRotY = serializedObject.FindProperty("LockRotationY");
            lockRotZ = serializedObject.FindProperty("LockRotationZ");

            // These might be null if [SerializeField] is missing in the main class
            bodyIndex = serializedObject.FindProperty("_bodyIndex");
            proxyIndex = serializedObject.FindProperty("_proxyIndex");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            // --- PHYSICAL PROPERTIES ---
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("Physical Properties", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(mass);
            EditorGUILayout.PropertyField(radius);
            EditorGUILayout.PropertyField(isKinematic);
            EditorGUILayout.EndVertical();

            // --- MATERIAL ---
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("Material (Elasticity)", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(materialCompliance);
            if (materialCompliance.floatValue > 0)
            {
                EditorGUILayout.HelpBox("Higher compliance = Softer collision response.", MessageType.Info);
            }

            EditorGUILayout.EndVertical();

            // --- CONSTRAINTS ---
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("Constraints", EditorStyles.boldLabel);

            DrawConstraintRow("Freeze Position", lockX, lockY, lockZ);
            DrawConstraintRow("Freeze Rotation", lockRotX, lockRotY, lockRotZ);

            EditorGUILayout.EndVertical();

            // --- DEBUG INFO ---
            if (Application.isPlaying)
            {
                EditorGUILayout.BeginVertical("box");
                EditorGUILayout.LabelField("Runtime Debug", EditorStyles.boldLabel);

                EditorGUI.BeginDisabledGroup(true);
                // SAFE GUARD: Check if properties exist before drawing
                if (bodyIndex != null) EditorGUILayout.PropertyField(bodyIndex, new GUIContent("Body ID"));
                if (proxyIndex != null) EditorGUILayout.PropertyField(proxyIndex, new GUIContent("Proxy ID"));
                EditorGUI.EndDisabledGroup();

                EditorGUILayout.EndVertical();
            }

            serializedObject.ApplyModifiedProperties();
        }

        // Robust Drawing Method
        private void DrawConstraintRow(string label, SerializedProperty x, SerializedProperty y, SerializedProperty z)
        {
            Rect rect = EditorGUILayout.GetControlRect();

            Rect labelRect = new Rect(rect.x, rect.y, EditorGUIUtility.labelWidth, rect.height);
            EditorGUI.LabelField(labelRect, label);

            float currentX = rect.x + EditorGUIUtility.labelWidth;
            float toggleWidth = 30f;

            float oldLabelW = EditorGUIUtility.labelWidth;
            int oldIndent = EditorGUI.indentLevel;

            EditorGUI.indentLevel = 0;
            EditorGUIUtility.labelWidth = 12f;

            Rect rectX = new Rect(currentX, rect.y, toggleWidth, rect.height);
            EditorGUI.PropertyField(rectX, x, new GUIContent("X"));

            Rect rectY = new Rect(currentX + toggleWidth, rect.y, toggleWidth, rect.height);
            EditorGUI.PropertyField(rectY, y, new GUIContent("Y"));

            Rect rectZ = new Rect(currentX + toggleWidth * 2, rect.y, toggleWidth, rect.height);
            EditorGUI.PropertyField(rectZ, z, new GUIContent("Z"));

            EditorGUIUtility.labelWidth = oldLabelW;
            EditorGUI.indentLevel = oldIndent;
        }
    }
}
#endif