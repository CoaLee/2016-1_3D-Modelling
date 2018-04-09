using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(BezierPatch))]
public class BezierPatchInspector : Editor {

	private BezierPatch bpatch;
	private Transform handleTransform;
	private Quaternion handleRotation;

	private const int lineSteps = 50;

	private void OnSceneGUI(){
		bpatch = target as BezierPatch;
		handleTransform = bpatch.transform;
		handleRotation = Tools.pivotRotation == PivotRotation.Local ? 
			handleTransform.rotation : Quaternion.identity;

		Vector3[] points = new Vector3[16];
		for (int i = 0; i < 16; i++) {
			points [i] = ShowPoint (i);
		}

		Handles.DrawBezier (points [0], points [12], points [4], points [8], Color.white, null, 2f);
		Handles.DrawBezier (points [12], points [15], points [13], points [14], Color.white, null, 2f);
		Handles.DrawBezier (points [15], points [3], points [11], points [7], Color.white, null, 2f);
		Handles.DrawBezier (points [3], points [0], points [2], points [1], Color.white, null, 2f);

		bpatch.Generate ();
	}

	private Vector3 ShowPoint (int index) {
		Vector3 point = handleTransform.TransformPoint(bpatch.points[index]);

		EditorGUI.BeginChangeCheck();
		point = Handles.DoPositionHandle(point, handleRotation);
		if (EditorGUI.EndChangeCheck()) {
			Undo.RecordObject(bpatch, "Move Point");
			EditorUtility.SetDirty(bpatch);
			bpatch.points[index] = handleTransform.InverseTransformPoint(point);
		}
		return point;
	}

	/*
	private void ShowDirections(){
		Handles.color = Color.green;
		Vector3 point = bpatch.GetPoint (0f);
		Handles.DrawLine(point, point + bpatch.GetNormal(0f, 0f));
		for (int i = 1; i <= lineSteps; i++) {
			point = bpatch.GetPoint(i / (float)lineSteps);
			Handles.DrawLine(point, point + bpatch.GetNormal(i / (float)lineSteps));
		}
	}*/
}