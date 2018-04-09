using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class SelectRectangle : MonoBehaviour {

	public enum MouseButton { left, right, middle }
	public enum SelectShape { Ring, Hexagon, RingBordered, HexagonBordered, Custom }
	public MouseButton mouseButton=MouseButton.left;
	public bool scaleToObject = true;
	public SelectShape selectShape = SelectShape.Ring;
	public Color selectColor=new Color(65f/255f, 120f/255f, 182.0f, 0.9f);
	public KeyCode multiSelectKey=KeyCode.LeftShift;
	public Color rectangleColor=new Color(190f/255f, 190f/255f, 1.0f, 0.2f);
	public Color rectangleBorderColor=new Color(0.5f, 0.5f, 1.0f, 0.9f);
	public GameObject onSelectUnit;
	public GameObject onDeselectUnit;
	public GameObject onStartSelect;
	public GameObject onEndSelect;
	public GameObject projectorPrefab;

	public List<GameObject> selectedObjects { get; set; }

	private Vector2 startPoint;
	private Texture2D rectTexture;
	private Texture2D borderTexture;
	private bool isDragging=false;
	private bool doDelete=false;
	private Texture2D circle, hexagon, circleb, hexagonb, custom;


	//Get Access to th list of all selected objects
	public List<GameObject> GetSelectedObjects() {
		return selectedObjects;
	}

	public void SelectObject(GameObject go) {
		SetObjectSelected(go, new Rect(0,0,Screen.width, Screen.height));
	}

	public void DeselectObject(GameObject go) {
		if (go.GetComponentsInChildren<Projector>().Length>0) {
			Projector prj = go.GetComponentsInChildren<Projector>()[0];
			GameObject prnt= prj.transform.parent.gameObject;
			Destroy(prnt);
			selectedObjects.Remove(go);
			//Send message if needed
			if(onDeselectUnit!=null) {
				onSelectUnit.SendMessage("OnDeselectUnit", go);
			}
		}
	}


	//Load base textures and creates needed ones
	void Awake() {
		rectTexture = CreateColorTexture (new Rect (0, 0, 2, 2), rectangleColor);
		borderTexture = CreateColorTexture (new Rect (0, 0, 2, 2), rectangleBorderColor);
		selectedObjects = new List<GameObject> ();
		circle = Resources.Load("selectring") as Texture2D;
		hexagon = Resources.Load("selecthex") as Texture2D;
		circleb = Resources.Load("selectring_border") as Texture2D;
		hexagonb = Resources.Load("selecthex_border") as Texture2D;
		custom = Resources.Load("custom") as Texture2D;
	}

	//Deselects all objects
	void DeselectAllObjects() {
		for(int i=0; i<selectedObjects.Count; i++) {
			Projector prj = selectedObjects[i].GetComponentsInChildren<Projector>()[0];
			GameObject prnt= prj.transform.parent.gameObject;
			Destroy(prnt);
			//Send messages if needed
			if(onDeselectUnit!=null) {
				onSelectUnit.SendMessage("OnDeselectUnit", selectedObjects[i]);
			}
		}
		selectedObjects.Clear();
	}

	//Check for mouse clicks and select rectangle
	void Update () {
		//If mouse select button is pressed
		if (Input.GetMouseButton(mouseButton.GetHashCode())) {
			//If start select
			if(!isDragging) {
				isDragging=true;
				doDelete=true;
				startPoint=Input.mousePosition;
				//Send message
				if(onStartSelect!=null) { 
					onSelectUnit.SendMessage("OnStartSelect", selectedObjects);
				}
			}
		}
		else if(isDragging) {
			//If ends select send message if needed
			isDragging=false;
			if(onEndSelect!=null) {
				onSelectUnit.SendMessage("OnEndSelect", selectedObjects);
			}
		}

		//If is dragging the select rectangle
		if (isDragging) {
			if(Vector2.Distance(startPoint, Input.mousePosition)<3)
				return;

			if(selectedObjects.Count>0 && doDelete) {
				//If not multiselect key pressed, clear the selected list
				if(!Input.GetKey(multiSelectKey)) {
					DeselectAllObjects();
					doDelete=false;
				}
			}
			Rect rect=new Rect(startPoint.x, Screen.height-startPoint.y, Input.mousePosition.x-startPoint.x, startPoint.y-Input.mousePosition.y);
			if(rect.width<0) {
				rect.x+=rect.width;
				rect.width*=-1;
			}
			if(rect.height<0) {
				rect.y+=rect.height;
				rect.height*=-1;
			}

			//Look for objects with TAG "selectable"
			try {
				GameObject[] allObjects = GameObject.FindGameObjectsWithTag("Selectable");
				foreach(GameObject go in allObjects) {
					SetObjectSelected(go, rect);
				}
			}
			catch (UnityException e) {
				Debug.Log(e.Message);
				Debug.LogWarning("ATTENTION!! Create a tag named 'Selectable' to avoid this errors!!");
			}

			//Look for objects sons of "Selectable" object
			GameObject objParent=GameObject.Find("Selectable");
			foreach (Transform child in objParent.transform) {
				SetObjectSelected(child.gameObject, rect);
			}
		}
	}

	//Test for a game object if needs to be selected
	void SetObjectSelected(GameObject go, Rect rect) {
		Vector3 pos = Camera.main.WorldToScreenPoint(go.transform.position);
		pos.y = Screen.height - pos.y;
		if(rect.Contains(new Vector2(pos.x, pos.y))) {
			if (go.GetComponentsInChildren<Projector>().Length==0) {
				selectedObjects.Add(go);				
				GameObject slc = (GameObject)Instantiate(projectorPrefab);
				Projector prj = slc.GetComponentsInChildren<Projector>()[0];
				if(scaleToObject) {
					Vector3 size=new Vector3(1,1,1);
					//First look for SkinnedMeshRenderer
					SkinnedMeshRenderer[] meshes = go.GetComponentsInChildren<SkinnedMeshRenderer>();
					if( meshes.Length > 0 ) {
						Bounds bounds = new Bounds( Vector3.zero, Vector3.zero );
						foreach( SkinnedMeshRenderer r in meshes ) {
							bounds.Encapsulate( r.sharedMesh.bounds );
						}
						size = bounds.size;
					}
					//If no skinned, look for mesh filters
					else {
						MeshFilter[] meshesf = go.GetComponentsInChildren<MeshFilter>();
						if( meshesf.Length > 0 ) {
							Bounds bounds = new Bounds( Vector3.zero, Vector3.zero );
							foreach( MeshFilter r in meshesf ) {
								bounds.Encapsulate( r.mesh.bounds );
							}
							size = bounds.size;
						}
					}
					prj.orthographicSize=Mathf.Max(go.transform.localScale.x*size.x/2.5f, go.transform.localScale.z*size.z/2.5f);
				}
				else
					prj.orthographicSize=0.7f;
				
				prj.material.SetColor("_Color", selectColor);
				if(selectShape==SelectShape.Ring)
					prj.material.SetTexture("_ShadowTex", circle);
				else if(selectShape==SelectShape.Hexagon)
					prj.material.SetTexture("_ShadowTex", hexagon);
				else if(selectShape==SelectShape.RingBordered)
					prj.material.SetTexture("_ShadowTex", circleb);
				else if(selectShape==SelectShape.HexagonBordered)
					prj.material.SetTexture("_ShadowTex", hexagonb);
				else if(selectShape==SelectShape.Custom)
					prj.material.SetTexture("_ShadowTex", custom);
				
				slc.transform.parent=go.transform;
				slc.transform.localPosition = new Vector3(0f, 0.1f, 0f);
				slc.transform.rotation = go.transform.rotation;
				//Send message if needed
				if(onSelectUnit!=null) {
					onSelectUnit.SendMessage("OnSelectUnit", go);
				}
			}
		}
		else if(!Input.GetKey(multiSelectKey)){
			DeselectObject(go);
		}
	}

	//Draws the select rectangle on screen
	void OnGUI() {
		if (isDragging) {
			Rect rect=new Rect(startPoint.x, Screen.height-startPoint.y, Input.mousePosition.x-startPoint.x, startPoint.y-Input.mousePosition.y);
			GUI.DrawTexture	(rect, rectTexture);
			GUI.DrawTexture	(new Rect(rect.x, rect.y, rect.width, 1), borderTexture);
			GUI.DrawTexture	(new Rect(rect.x, rect.y, 1, rect.height), borderTexture);
			GUI.DrawTexture	(new Rect(rect.x+rect.width, rect.y, 1, rect.height), borderTexture);
			GUI.DrawTexture	(new Rect(rect.x, rect.y+rect.height, rect.width,1), borderTexture);
		}
	}

	Texture2D CreateColorTexture(Rect rect, Color color) {
		Texture2D result = new Texture2D((int)rect.width, (int)rect.height, TextureFormat.ARGB32, false);
		int y = 0;
		while (y < result.height) {
			int x = 0;
			while (x < result.width) {
				result.SetPixel(x, y, color);
				++x;
			}
			++y;
		}
		result.Apply();
		return result;
	}
}
