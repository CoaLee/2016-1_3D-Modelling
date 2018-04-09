using UnityEngine;
using System.Collections;

public class CameraPlayer : MonoBehaviour {

	public float speed = 1f;

	public float rotAccel = 0.1f;
	public float rotSpeed = 1f;
	public float rotDistance = 15f;
	public bool rotCam = true;

	public Vector3 rotOrigin;

	void Start () {
		rotOrigin = new Vector3 ();
	}
	
	void Update () {
		Vector3 movement = Vector3.zero;
		if (Input.GetKeyDown (KeyCode.A)) {
			movement = -transform.right;
		}
		if (Input.GetKeyDown (KeyCode.D)) {
			movement = transform.right;
		}
		if (Input.GetKeyDown (KeyCode.E)) {
			movement = transform.forward;
		}
		if (Input.GetKeyDown (KeyCode.C)) {
			movement = -transform.forward;
		}
		if (Input.GetKeyDown (KeyCode.W)) {
			movement = transform.up;
		}
		if (Input.GetKeyDown (KeyCode.S	)) {
			movement = -transform.up;
		}
		if(Input.GetKey(KeyCode.X)){
			transform.Rotate (transform.up * speed);
		}
		if (Input.GetKey(KeyCode.Z)) {
			transform.Rotate (-transform.up * speed);
		}

		/* Camear Rotation */
		if (Input.GetKey (KeyCode.Y)) {
			float rotTheta = Mathf.Deg2Rad * rotSpeed;
			Vector3 tmpV = transform.position;
			tmpV.y = rotOrigin.y;
			tmpV = rotOrigin - tmpV;
			float modDistance = rotDistance * tmpV.magnitude / Vector3.Distance (transform.position, rotOrigin);
			transform.position += (tmpV.normalized * (modDistance * (1.0f - Mathf.Cos(rotTheta))));
			transform.position += (transform.right * (modDistance * Mathf.Sin (rotTheta)));
			transform.Rotate (-Vector3.up * rotSpeed, Space.World);
		}
		if (Input.GetKey (KeyCode.T)) {
			float rotTheta = Mathf.Deg2Rad * rotSpeed;
			Vector3 tmpV = transform.position;
			tmpV.y = rotOrigin.y;
			tmpV = rotOrigin - tmpV;
			float modDistance = rotDistance * tmpV.magnitude / Vector3.Distance (transform.position, rotOrigin);
			transform.position += (tmpV.normalized * (modDistance * (1.0f - Mathf.Cos(rotTheta))));
			transform.position -= (transform.right * (modDistance * Mathf.Sin (rotTheta)));
			transform.Rotate (Vector3.up * rotSpeed, Space.World);
		}
		if (Input.GetKey (KeyCode.H)) {
			float rotTheta = Mathf.Deg2Rad * rotSpeed;
			transform.position += (transform.forward * (rotDistance * (1.0f - Mathf.Cos(rotTheta))));
			transform.position += (transform.up * (rotDistance * Mathf.Sin (rotTheta)));
			transform.Rotate (Vector3.right * rotSpeed);
		}
		if (Input.GetKey (KeyCode.G)) {
			float rotTheta = Mathf.Deg2Rad * rotSpeed;
			transform.position += (transform.forward * (rotDistance * (1.0f - Mathf.Cos(rotTheta))));
			transform.position -= (transform.up * (rotDistance * Mathf.Sin (rotTheta)));
			transform.Rotate (-Vector3.right * rotSpeed);
		}
		if (Input.GetKey (KeyCode.N)) {
			transform.Rotate (-Vector3.forward * rotSpeed);
		}
		if (Input.GetKey (KeyCode.B)) {
			transform.Rotate (Vector3.forward * rotSpeed);
		}
		if (Input.GetKey (KeyCode.R)) {
			transform.rotation = Quaternion.identity;
			transform.position = new Vector3 (0f, 0f, 10f);
		}


		transform.position += movement * speed;
	}

	private void OnDrawGizmos(){
		Gizmos.color = Color.white;
		Gizmos.DrawLine (transform.position, transform.position + transform.forward * rotDistance);

		Gizmos.color = Color.yellow;
		Gizmos.DrawLine (transform.position, transform.position + transform.forward * rotDistance);
	}
}