using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class EventReceiverSample : MonoBehaviour {

	//Sample class to receive events from select tool asset
	//Use as a base code to create your own handler
	//Simply drop on any gameobject in scene

	public void OnSelectUnit(GameObject go) {
		Debug.Log ("Unit "+go.name+" selected");
	}
	public void OnDeselectUnit(GameObject go) {
		Debug.Log ("Unit "+go.name+" deselected");
	}

	public void OnStartSelect(List<GameObject> selectedObjects) {
		Debug.Log ("Start a new selection ("+selectedObjects.Count+" objects already selected)");
	}

	public void OnEndSelect(List<GameObject> selectedObjects) {
		Debug.Log ("Finish selection ("+selectedObjects.Count+" objects already selected)");
		for(int i=0; i<selectedObjects.Count; i++)
			Debug.Log(selectedObjects[i].name+" object is selected");
	}
}
