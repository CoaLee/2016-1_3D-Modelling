using UnityEngine;
using System.Collections;

public class MouseInput : MonoBehaviour {	
	public Mesh mesh;
	public Material material;

	private GameObject pointer;
	private bool isUsingPointer = false;

	private GameObject obj_root; /* objs[0]: root of all objects */

	private GameObject[] objs;
	private int obj_num = 0;

	private float pointDistanceOrth = 5f;
	private float pointDistancePers = 15f;

	private int[] point;
	private int point_num = 0;
	private Vector3[] point_pos;

	private int[] bcurve;
	private int bcurve_num = 0;
	private int bcurve_tmpcnt = 0;

	private int[] bpatch;
	private int bpatch_num = 0;
	private int bpatch_tmpcnt = 0;

	public float snapsize = 10f;

	enum WorkingMode {IDLE, POINT, BCURVE, BPATCH} ;
	private WorkingMode working_mode = WorkingMode.IDLE;

	void Start(){
		pointer = GameObject.FindGameObjectWithTag ("Pointer");
		obj_root = new GameObject ("obj_root");
		objs = new GameObject[400];
		point = new int[200];
		point_pos = new Vector3[200];
		bcurve = new int[100];
		bpatch = new int[100];
	}

	/* Handling event in every frame */ 
	void Update () {
		if (Input.GetMouseButtonDown (0)) {
			HandleInput ();
		}

		if (Input.GetKeyDown (KeyCode.Space)) {
			isUsingPointer = true;
			HandleInput ();
		}

		/* Key V : View Switch between Perspective & Orthographic */
		if(Input.GetKeyUp(KeyCode.V)){
			Camera.main.orthographic = !Camera.main.orthographic;
		}

		/* Mode Triggering & Changing. TODO: trigger it by clicking UI panels */
		if (working_mode == WorkingMode.IDLE) {
			if (Input.GetKeyUp (KeyCode.Alpha1)) {
				working_mode = WorkingMode.POINT;
				GameObject.FindWithTag ("modeIndicator").GetComponent<UnityEngine.UI.Text> ().text = "Mode: point";
			} else if (Input.GetKeyUp (KeyCode.Alpha2)) {
				working_mode = WorkingMode.BCURVE;
				GameObject.FindWithTag ("modeIndicator").GetComponent<UnityEngine.UI.Text> ().text = "Mode: bcurve";
			} else if (Input.GetKeyUp (KeyCode.Alpha3)) {
				working_mode = WorkingMode.BPATCH;
				GameObject.FindWithTag ("modeIndicator").GetComponent<UnityEngine.UI.Text> ().text = "Mode: bpatch";
			}
		} else {
			if (Input.GetKeyUp (KeyCode.Alpha1) ||
				Input.GetKeyUp (KeyCode.Alpha2) ||
				Input.GetKeyUp (KeyCode.Alpha3)) {
				ModeCancelling();
				working_mode = WorkingMode.IDLE;
				GameObject.FindWithTag ("modeIndicator").GetComponent<UnityEngine.UI.Text> ().text = "Mode: idle";
			}
		}
	}

	void HandleInput(){
		switch (working_mode) {
		case WorkingMode.IDLE:
			break;

		case WorkingMode.POINT:
			HandlePoint ();
			break;

		case WorkingMode.BCURVE:
			HandleBcurve ();
			break;

		case WorkingMode.BPATCH:
			HandleBpatch ();
			break;

		default:
			break;
		}
		isUsingPointer = false;
	}

	void HandlePoint(){
		Vector3 pos;

		/* snap checking: comparing distance between (x, y)s with snapsize, ignoring depth(z element).  */
		if (Camera.main.orthographic == true) {
			if (!isUsingPointer) {
				pos = Camera.main.ScreenToWorldPoint (Input.mousePosition);
				pos += transform.forward * pointDistanceOrth;
			} else {
				pos = pointer.transform.position;
			}

			Vector3 tmpP;
			Vector3 tmpM = Camera.main.WorldToScreenPoint (pos);
			tmpM.z = 0;

			if (Input.GetKey (KeyCode.LeftControl)) {
				for (int i = 0; i < point_num; i++) {
					tmpP = Camera.main.WorldToScreenPoint (point_pos [i]);
					tmpP.z = 0;

					if ((tmpP - tmpM).magnitude < snapsize/2) {
						Debug.Log ("snapped!" + (tmpP - tmpM).magnitude + ", " + snapsize);
						pos = point_pos [i];
					}
				}
			}
		} else {
			if (!isUsingPointer) {
				Ray ray = Camera.main.ScreenPointToRay (Input.mousePosition);
				Plane xy = new Plane (transform.forward, transform.position + transform.forward * pointDistancePers);
				float distance;
				xy.Raycast (ray, out distance);
				pos = ray.GetPoint (distance);
			} else {
				pos = pointer.transform.position;
			}

			Vector3 tmpP;
			Vector3 tmpM = Camera.main.WorldToScreenPoint (pos);
			tmpM.z = 0;

			if (Input.GetKey (KeyCode.LeftControl)) {
				for (int i = 0; i < point_num; i++) {
					tmpP = Camera.main.WorldToScreenPoint (point_pos [i]);
					tmpP.z = 0;

					if ((tmpP - tmpM).magnitude < snapsize) {
						Debug.Log ("snapped!" + (tmpP - tmpM).magnitude + ", " + snapsize);
						pos = point_pos [i];
					}
				}
			}
		}

		GameObject gm = new GameObject ("point [" + point_num + "]");
		gm.transform.SetParent (obj_root.transform);
		gm.AddComponent<MeshFilter> ().mesh = mesh;
		gm.AddComponent<MeshRenderer> ().material = material;
		gm.AddComponent<Battlehub.RTEditor.ExposeToEditor> ();
		gm.transform.position = pos;
		gm.transform.localScale = new Vector3 (0.05f, 0.05f, 0.05f);

		point_pos[point_num] = pos;

		point [point_num++] = obj_num;
		objs [obj_num++] = gm;
	}

	void HandleBcurve(){
		bcurve_tmpcnt++;
		HandlePoint ();

		if(bcurve_tmpcnt == 4){
			GameObject bobject = new GameObject ("B curve " + bcurve_num);
			bobject.transform.SetParent (obj_root.transform);
			bobject.AddComponent<Battlehub.RTEditor.ExposeToEditor> ();

			Vector3[] tmpPoints = new Vector3[4];
			for (int i = 0; i < 4; i++)
				tmpPoints [i] = point_pos [point_num - 4 + i];
			bobject.AddComponent<BezierCurve>().points = tmpPoints;

			for (int i = 1; i <= 4; i++)
				objs [point [point_num - i]].transform.SetParent (bobject.transform);
			
			bcurve_tmpcnt = 0;

			bcurve [bcurve_num++] = obj_num;
			objs [obj_num++] = bobject;
		}
	}

	void HandleBpatch(){
		bpatch_tmpcnt++;
		HandleBcurve ();

		if (bpatch_tmpcnt == 16) {
			GameObject bobject = new GameObject ("B patch " + bpatch_num);
			bobject.transform.SetParent (obj_root.transform);
			bobject.AddComponent<Battlehub.RTEditor.ExposeToEditor> ();

			bobject.AddComponent<BezierPatch> ().
			BCSet (objs[bcurve[bcurve_num - 4]].GetComponent<BezierCurve>(), 
				objs[bcurve [bcurve_num - 3]].GetComponent<BezierCurve>(), 
				objs[bcurve [bcurve_num - 2]].GetComponent<BezierCurve>(), 
				objs[bcurve [bcurve_num - 1]].GetComponent<BezierCurve>());
	
			for (int i = 1; i <= 4; i++)
				objs[bcurve [bcurve_num - i]].transform.SetParent (bobject.transform);

			bpatch_tmpcnt = 0;

			bpatch [bpatch_num++] = obj_num;
			objs [obj_num++] = bobject;
		}
	}

	void ModeCancelling(){
		/* TODO removing points & bcurve , ... during construction */

	}

	void OnPostRender(){
		for (int i = 0; i < bcurve_num; i++) {
			objs[bcurve [i]].GetComponent<BezierCurve>().Drawing ();
		}
		//bpatch.Drawing ();
	}
}