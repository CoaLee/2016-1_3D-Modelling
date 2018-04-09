using UnityEngine;
using System.Collections;

public class CameraPlayer : MonoBehaviour {
    public float zoomSensitivity = 1f;
	public float rotDistance = 15f;
	public bool rotCam = true;
	public float rotSpeed = 1f;
	Vector3 rotOrigin = new Vector3();

	void Start () {
	}
	
	void Update () {
		/* Camear Rotation */
		if (Input.GetKey (KeyCode.Y)) {
			float rotTheta = Mathf.Deg2Rad * rotSpeed;
			Vector3 tmpV = transform.position;
			tmpV.y = rotOrigin.y;
			tmpV = rotOrigin - tmpV;
			float modDistance = rotDistance * tmpV.magnitude / Vector3.Distance (transform.position, rotOrigin);

			if (transform.up.y >= 0) {
				transform.position += (tmpV.normalized * (modDistance * (1.0f - Mathf.Cos (rotTheta))));
				transform.position += (transform.right * (modDistance * Mathf.Sin (rotTheta)));
			} else {
				transform.position -= (tmpV.normalized * (modDistance * (1.0f - Mathf.Cos (rotTheta))));
				transform.position -= (transform.right * (modDistance * Mathf.Sin (rotTheta)));
			}
			transform.Rotate (-Vector3.up * rotSpeed, Space.World);
		}
		if (Input.GetKey (KeyCode.T)) {
			float rotTheta = Mathf.Deg2Rad * rotSpeed;
			Vector3 tmpV = transform.position;
			tmpV.y = rotOrigin.y;
			tmpV = rotOrigin - tmpV;
			float modDistance = rotDistance * tmpV.magnitude / Vector3.Distance (transform.position, rotOrigin);

			if (transform.up.y >= 0) {
				transform.position += (tmpV.normalized * (modDistance * (1.0f - Mathf.Cos (rotTheta))));
				transform.position -= (transform.right * (modDistance * Mathf.Sin (rotTheta)));
			} else {
				transform.position -= (tmpV.normalized * (modDistance * (1.0f - Mathf.Cos (rotTheta))));
				transform.position += (transform.right * (modDistance * Mathf.Sin (rotTheta)));
			}
				
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
		if (Input.GetKey (KeyCode.R)) {
			transform.rotation = Quaternion.identity;
			transform.position = new Vector3 (0f, 0f, -15f);
		}
	}

    public void rotateBySensor(Vector3 rotateVector)
    {
        if (Input.GetKey(KeyCode.X))
        {
            //Yaw
            /*
            float rotThetaX = Mathf.Deg2Rad * rotateVector.x;
            Vector3 modifiedForward = transform.forward;
            modifiedForward.y = 0;
            float modifiedDistance = rotDistance * Mathf.Cos(Mathf.Deg2Rad * transform.rotation.eulerAngles.x);
            
            transform.position += (modifiedForward * (modifiedDistance * (1.0f - Mathf.Cos(rotThetaX))));
            transform.position += (transform.right * (modifiedDistance * Mathf.Sin(rotThetaX)));
            transform.Rotate(-Vector3.up * rotateVector.x);
            */

            float rotThetaX = Mathf.Deg2Rad * rotateVector.x;
			Vector3 tmpV = transform.position;
			tmpV.y = rotOrigin.y;
			tmpV = rotOrigin - tmpV;
			float modDistance = rotDistance * tmpV.magnitude / Vector3.Distance (transform.position, rotOrigin);

			if (transform.up.y >= 0) {
				transform.position += (tmpV.normalized * (modDistance * (1.0f - Mathf.Cos (rotThetaX))));
				transform.position += (transform.right * (modDistance * Mathf.Sin (rotThetaX)));
			} else {
				transform.position -= (tmpV.normalized * (modDistance * (1.0f - Mathf.Cos (rotThetaX))));
				transform.position -= (transform.right * (modDistance * Mathf.Sin (rotThetaX)));
			}
            transform.Rotate(-Vector3.up * rotateVector.x, Space.World);

            //Pitch
            float rotThetaY = Mathf.Deg2Rad * rotateVector.y;
            transform.position += (transform.forward * (rotDistance * (1.0f - Mathf.Cos(rotThetaY))));
            transform.position += (transform.up * (rotDistance * Mathf.Sin(rotThetaY)));
            transform.Rotate(Vector3.right * rotateVector.y);
            
            //Roll
            //transform.Rotate(Vector3.forward, rotateVector.z);
        }

        if (Input.GetKey(KeyCode.Z))
        {
            transform.position += transform.forward * rotateVector.y * zoomSensitivity;
            rotDistance -= rotateVector.y * zoomSensitivity;
        }
    }


	private void OnDrawGizmos(){
		Gizmos.color = Color.white;
		Gizmos.DrawLine (transform.position, transform.position + transform.forward * rotDistance);

        /*
        Gizmos.color = Color.yellow;
        Vector3 modifiedForward = transform.forward;
        modifiedForward.y = 0;
        float modifiedDistance = rotDistance * Mathf.Cos(Mathf.Deg2Rad * transform.rotation.eulerAngles.x);
            
        Gizmos.DrawLine(transform.position, transform.position + modifiedForward * modifiedDistance);*/
	}
}