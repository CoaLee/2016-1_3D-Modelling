using UnityEngine;

public static class Bezier2D {

	public static Vector3 GetPoint(Vector3 [] points, float u, float v){
		u = Mathf.Clamp01 (u);
		v = Mathf.Clamp01 (v);
		float oneMinusU = 1f - u;
		float oneMinusV = 1f - v;

		float co_u1 = oneMinusU * oneMinusU * oneMinusU;
		float co_u2 = 3f * oneMinusU * oneMinusU * u;
		float co_u3 = 3f * oneMinusU * u * u;
		float co_u4 = u * u * u;

		float co_v1 = oneMinusV * oneMinusV * oneMinusV;
		float co_v2 = 3f * oneMinusV * oneMinusV * v;
		float co_v3 = 3f * oneMinusV * v * v;
		float co_v4 = v * v * v;

		Vector3 i1 = co_u1 * points [0] + co_u2 * points [4] + co_u3 * points [8] + co_u4 * points [12];
		Vector3 i2 = co_u1 * points [1] + co_u2 * points [5] + co_u3 * points [9] + co_u4 * points [13];
		Vector3 i3 = co_u1 * points [2] + co_u2 * points [6] + co_u3 * points [10] + co_u4 * points [14];
		Vector3 i4 = co_u1 * points [3] + co_u2 * points [7] + co_u3 * points [11] + co_u4 * points [15];

		return co_v1 * i1 + co_v2 * i2 + co_v3 * i3 + co_v4 * i4;
	}

	public static Vector3 GetNormal(Vector3 [] points, float u, float v){
		u = Mathf.Clamp01 (u);
		v = Mathf.Clamp01 (v);
		float oneMinusU = 1f - u;
		float oneMinusV = 1f - v;

		float co_u1 = oneMinusU * oneMinusU * oneMinusU;
		float co_u2 = 3f * oneMinusU * oneMinusU * u;
		float co_u3 = 3f * oneMinusU * u * u;
		float co_u4 = u * u * u;

		float d_u1 = -(3f * oneMinusU * oneMinusU);
		float d_u2 = 3f * oneMinusU * (1f - 3f * u);
		float d_u3 = 3f * u * (2f - 3f * u);
		float d_u4 = 3f * u * u;

		float co_v1 = oneMinusV * oneMinusV * oneMinusV;
		float co_v2 = 3f * oneMinusV * oneMinusV * v;
		float co_v3 = 3f * oneMinusV * v * v;
		float co_v4 = v * v * v;

		float d_v1 = -(3f * oneMinusV * oneMinusV);
		float d_v2 = 3f * oneMinusV * (1f - 3f * v);
		float d_v3 = 3f * v * (2f - 3f * v);
		float d_v4 = 3f * v * v;

		Vector3 iu1 = d_u1 * points [0] + d_u2 * points [4] + d_u3 * points [8] + d_u4 * points [12];
		Vector3 iu2 = d_u1 * points [1] + d_u2 * points [5] + d_u3 * points [9] + d_u4 * points [13];
		Vector3 iu3 = d_u1 * points [2] + d_u2 * points [6] + d_u3 * points [10] + d_u4 * points [14];
		Vector3 iu4 = d_u1 * points [3] + d_u2 * points [7] + d_u3 * points [11] + d_u4 * points [15];

		Vector3 du = co_v1 * iu1 + co_v2 * iu2 + co_v3 * iu3 + co_v4 * iu4;

		Vector3 iv1 = d_v1 * points [0] + d_v2 * points [1] + d_v3 * points [2] + d_v4 * points [3];
		Vector3 iv2 = d_v1 * points [4] + d_v2 * points [5] + d_v3 * points [6] + d_v4 * points [7];
		Vector3 iv3 = d_v1 * points [8] + d_v2 * points [9] + d_v3 * points [10] + d_v4 * points [11];
		Vector3 iv4 = d_v1 * points [12] + d_v2 * points [13] + d_v3 * points [14] + d_v4 * points [15];

		Vector3 dv = co_u1 * iv1 + co_u2 * iv2 + co_u3 * iv3 + co_u4 * iv4;

		return Vector3.Cross (du, dv).normalized;
	}
}
