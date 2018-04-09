using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class BezierPatch : MonoBehaviour {

	public Vector3[] points;

	private Mesh mesh;
	private Vector3[] vertices;
	private Vector3[] normals;
	public Material mat;
	public Shader shdr;
	private const int drawstep = 10;

	public bool MODE_TWOSIDE = false;

	public Vector3 GetPoint(float u, float v){
		return transform.TransformPoint (Bezier2D.GetPoint (points, u, v));
	}

	public Vector3 GetNormal(float u, float v){
		return transform.TransformPoint (Bezier2D.GetNormal (points, u, v));
	}

	public void Reset(){
		points = new Vector3[16];
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				points [i * 4 + j] = new Vector3 ((float)i, (float)j);
			}
		}
	}

	/* later, need to check the normal vector of surface, which should be opposite direction with seeing vector... */ 
	public void BCSet(BezierCurve bc1, BezierCurve bc2, BezierCurve bc3, BezierCurve bc4){
		Vector3[] tmppoints = new Vector3[16];
		Vector3[] edge1 = new Vector3[4];
		Vector3[] edge2 = new Vector3[4];
		Vector3[] edge3 = new Vector3[4];
		Vector3[] edge4 = new Vector3[4];

		/* need to think a general algorithm. for now just simple.
		edge1 = bc1.points;
		if (bc2.points [0] == edge1 [3]) {
			edge2 = bc2.points;
		} else if (bc2.points [3] == edge1 [3]) {
			for (int i = 0; i < 4; i++) {
				edge2 [i] = bc2.points [3 - i];
			}
			bc2 = 
		}
		*/

		edge1 = bc1.points;
		edge2 = bc2.points;
		edge3 = bc3.points;
		edge4 = bc4.points;

		/* 12 13 14 15
		 * 08 09 10 11
		 * 04 05 06 07
		 * 00 01 02 03
		 * */
		tmppoints [0] = edge1 [3];
		tmppoints [1] = edge1 [2];
		tmppoints [2] = edge1 [1];
		tmppoints [3] = edge1 [0];

		tmppoints [4] = edge2 [1];
		tmppoints [8] = edge2 [2];

		tmppoints [12] = edge3 [0];
		tmppoints [13] = edge3 [1];
		tmppoints [14] = edge3 [2];
		tmppoints [15] = edge3 [3];

		tmppoints [11] = edge4 [1];
		tmppoints [7] = edge4 [2];

		tmppoints [5] = (2f * tmppoints [0] + tmppoints [15]) / 3;
		tmppoints [6] = (2f * tmppoints [3] + tmppoints [12]) / 3;
		tmppoints [9] = (2f * tmppoints [12] + tmppoints [3]) / 3;
		tmppoints [10] = (2f * tmppoints [15] + tmppoints [0]) / 3;

		points = tmppoints;

		Generate ();
	}
		
	void Update(){
		
	}

	void Awake(){
		if (!MODE_TWOSIDE) {
			vertices = new Vector3[(drawstep + 1) * (drawstep + 1)];
			normals = new Vector3[(drawstep + 1) * (drawstep + 1)];
		} else {
			vertices = new Vector3[((drawstep + 1) * (drawstep + 1)) * 2 ];
			normals = new Vector3[((drawstep + 1) * (drawstep + 1)) * 2 ];
		}

		Reset ();
		Generate ();
		if (!mat) {
			mat = new Material (Shader.Find ("Custom/BPatchShader"));
		}
	}

	public void Generate(){
		GetComponent<MeshFilter> ().mesh = mesh = new Mesh ();
		mesh.name = "Bezier Patch";
		MeshRenderer renderer = transform.GetComponent<MeshRenderer> ();
		renderer.material = mat;
		CreateVertices ();
		CreateTriangles ();
	}

	private void CreateVertices(){
		int v = 0;
		for (int i = 0; i <= drawstep; i++) {
			for (int j = 0; j <= drawstep; j++) {
				normals [v] = Bezier2D.GetNormal (points, (float)i / drawstep, (float)j / drawstep);
				vertices [v++] = Bezier2D.GetPoint (points, (float)i / drawstep, (float)j / drawstep);
			}
		}
		if (MODE_TWOSIDE) {
			for (int i = drawstep; i >= 0; i--) {
				for (int j = drawstep; j >= 0; j--) {
					normals [v] = -1 * Bezier2D.GetNormal (points, (float)i / drawstep, (float)j / drawstep);
					vertices [v++] = Bezier2D.GetPoint (points, (float)i / drawstep, (float)j / drawstep);
				}
			}
		}
		mesh.vertices = vertices;
		mesh.normals = normals;
	}

	private void CreateTriangles(){
		int[] triangles;
		if (!MODE_TWOSIDE)
			triangles = new int[drawstep * drawstep * 6];
		else
			triangles = new int[drawstep * drawstep * 6 * 2];
		int t = 0, v = 0;
		for (int i = 0; i < drawstep; i++, v++) {
			for (int j = 0; j < drawstep; j++, v++) {
				t = SetQuad (triangles, t, v, v + 1, v + drawstep + 1, v + drawstep + 2);
			}
		}
		if (MODE_TWOSIDE) {
			v += drawstep;
			for (int i = 0; i < drawstep; i++, v++) {
				for (int j = 0; j < drawstep; j++, v++) {
					t = SetQuad (triangles, t, v, v + 1, v + drawstep + 1, v + drawstep + 2);
				}
			}
		}
		mesh.triangles = triangles;
	}

	private int SetQuad(int[] triangles, int i, int v00, int v10, int v01, int v11){
		triangles [i] = v00;
		triangles [i + 1] = triangles [i + 4] = v01;
		triangles [i + 2] = triangles [i + 3] = v10;
		triangles [i + 5] = v11;
		return i + 6;
	}

	private void OnDrawGizmos(){
		if (points == null) {
			return;
		}

		Gizmos.color = Color.yellow;
		for (int i = 0; i < points.Length; i++) {
			Gizmos.DrawSphere (transform.position+points [i], 0.05f);
		}
		for (int i = 0; i < vertices.Length; i++) {
			Gizmos.color = Color.yellow;
			Gizmos.DrawRay (vertices [i], normals [i]);
		}
	}
}
