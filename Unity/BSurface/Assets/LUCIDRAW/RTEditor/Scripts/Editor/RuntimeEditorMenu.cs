using UnityEngine;
using UnityEditor;
using System.Linq;
using UnityEngine.EventSystems;
using System.Collections.Generic;

namespace Battlehub.RTEditor
{
    public static class RTEditorMenu
    {
        const string root = "Battlehub/RTEditor/";

        [MenuItem("Tools/Runtime Editor/Create")]
        public static void CreateRuntimeEditor()
        {
            if (!Object.FindObjectOfType<EventSystem>())
            {
                GameObject es = new GameObject();
                es.AddComponent<EventSystem>();
                es.AddComponent<StandaloneInputModule>();
                es.name = "EventSystem";
            }
            Undo.RegisterCreatedObjectUndo(InstantiateRuntimeEditor(), "Battlehub.RTEditor.Create");
        }

        public static GameObject InstantiateRuntimeEditor()
        {
            return InstantiatePrefab("RuntimeEditor.prefab");
        }

        [MenuItem("Tools/Runtime Editor/Expose To Editor", validate = true)]
        private static bool CanExposeToEditor()
        {

            return Selection.gameObjects != null && Selection.gameObjects.Length > 0 &&
                Object.FindObjectOfType<RuntimeEditor>();
        }

        [MenuItem("Tools/Runtime Editor/Expose To Editor")]
        private static void ExposeToEditor()
        {
            RuntimeEditor editor = Object.FindObjectOfType<RuntimeEditor>();
            Undo.RecordObject(editor, "Battlehub.RTEditor.ExposeToEditor");
            List<GameObject> prefabs = new List<GameObject>(editor.Prefabs);

            foreach (GameObject go in Selection.gameObjects)
            {
                if(RuntimePrefabs.IsPrefab(go.transform))
                {
                    prefabs.Add(go);
                }
                else
                {
                    Undo.RegisterCreatedObjectUndo(go.AddComponent<ExposeToEditor>(), "Battlehub.RTEditor.ExposeToEditor");
                }
            }

            editor.Prefabs = prefabs.ToArray();

            EditorUtility.SetDirty(editor);
        }

        [MenuItem("Tools/Runtime Editor/Hide From Editor", validate = true)]
        private static bool CanHideFromEditor()
        {
            return Selection.gameObjects != null && Selection.gameObjects.Length > 0;
        }

        [MenuItem("Tools/Runtime Editor/Hide From Editor")]
        private static void HideFromEditor()
        {
            RuntimeEditor editor = Object.FindObjectOfType<RuntimeEditor>();
            List<GameObject> prefabs;
            if (editor != null)
            {
                Undo.RecordObject(editor, "Battlehub.RTEditor.ExposeToEditor");
                prefabs = new List<GameObject>(editor.Prefabs);
            }
            else
            {
                prefabs = new List<GameObject>();
            }
            
            foreach (GameObject go in Selection.gameObjects)
            {
                if (RuntimePrefabs.IsPrefab(go.transform))
                {
                    prefabs.Remove(go);
                }
                else
                {
                    ExposeToEditor exposeToEditor = go.GetComponent<ExposeToEditor>();
                    if (exposeToEditor != null)
                    {
                        Undo.DestroyObjectImmediate(exposeToEditor);
                    }
                }
            }

            if(editor != null)
            {
                editor.Prefabs = prefabs.ToArray();
            }

            EditorUtility.SetDirty(editor);
        }

        public static GameObject InstantiatePrefab(string name)
        {
            Object prefab = AssetDatabase.LoadAssetAtPath("Assets/" + root + "Prefabs/" + name, typeof(GameObject));
            return (GameObject)PrefabUtility.InstantiatePrefab(prefab);
        }
    }

}
