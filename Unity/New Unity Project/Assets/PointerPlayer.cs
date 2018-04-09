using UnityEngine;
using System.Collections;

public class PointerPlayer : MonoBehaviour {

	private float speed = 0;
	public float MAX_SPEED = 0.07f;
	public float accel = 0.001f;
    private Vector3 originalPosition;

	// Use this for initialization
	void Start () {
        originalPosition = transform.position;
	}

	// Update is called once per frame
	void Update () {
		Vector3 movement = Vector3.zero;
		if (Input.GetKey (KeyCode.J)) {
			movement = -transform.right;
			if(speed < MAX_SPEED) speed += accel;
		}
		if (Input.GetKey (KeyCode.L)) {
			movement = transform.right;
			if(speed < MAX_SPEED) speed += accel;
		}
		if (Input.GetKey (KeyCode.I)) {
			movement = transform.up;
			if(speed < MAX_SPEED) speed += accel;
		}
		if (Input.GetKey (KeyCode.K)) {
			movement = -transform.up;
			if(speed < MAX_SPEED) speed += accel;
		}
		if (Input.GetKey (KeyCode.U)) {
			movement = transform.forward;
			if(speed < MAX_SPEED) speed += accel;
		}
		if (Input.GetKey (KeyCode.M)) {
			movement = -transform.forward;
			if(speed < MAX_SPEED) speed += accel;
		}
		if (Input.GetKeyUp (KeyCode.J) ||
		    Input.GetKeyUp (KeyCode.L) ||
		    Input.GetKeyUp (KeyCode.I) ||
		    Input.GetKeyUp (KeyCode.K) ||
		    Input.GetKeyUp (KeyCode.U) ||
		    Input.GetKeyUp (KeyCode.M)) {
			speed = 0;
		}
		if (Input.GetKeyDown (KeyCode.LeftControl) && Input.GetKeyDown (KeyCode.Z)) {
			
		}
		transform.position += movement * speed;
	}

    public void moveBySensor(Vector3 moveVector)
    {
        transform.position += moveVector * Time.smoothDeltaTime;
    }

    public void positionBySensor(Vector3 positionVector)
    {
        transform.position = originalPosition + positionVector;
    }
}