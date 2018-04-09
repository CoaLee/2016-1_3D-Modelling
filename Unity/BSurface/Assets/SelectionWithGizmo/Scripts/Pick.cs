using UnityEngine;
using System.Collections;

public class Pick : MonoBehaviour {
	enum Mode{SELECT,OPER};
	Mode operMode;
	Rect drawRect=new Rect();
	bool drawSelectRect=false;
	GameObject   objDummy;
	ArrayList arya=new ArrayList();
	GameObject[] objChoosen;
	Shader[]   matChoosen;
	Transform [] objParent;
	public Material lineMat;
	public GameObject   lineShader;

	void Start () {
	
	/*lineMat=new Material( "Shader \"Lines/Colored Blended\" {" +
		                          "SubShader { Pass { " +
		                          "    Blend SrcAlpha OneMinusSrcAlpha " +
		                          "    ZWrite Off Cull Off Fog { Mode Off } " +
		                          "    BindChannels {" +
		                          "      Bind \"vertex\", vertex Bind \"color\", color }" +
		                          "} } }" );*/
		objDummy=new GameObject("Dummy");
		operMode=Mode.SELECT;
	//	lineMat.SetColor("_Color",new Color(1f,1f,1f,0.2f));
	}
	
	void Update () {
		if(Input.GetMouseButtonDown(0) && operMode==Mode.SELECT)
		{
			drawRect.x=Input.mousePosition.x;
			drawRect.y=Input.mousePosition.y;
			drawSelectRect=true;
				
		}
		if(Input.GetMouseButtonDown(1))
		{
			operMode=Mode.SELECT;
			
			GizmoRender gzr= objDummy.GetComponent<GizmoRender>();
			Destroy(gzr);
			for(int i=0;i<objChoosen.Length;i++)
			{
				if(objChoosen[i]!=null)
				{
					if(objChoosen[i].GetComponent<Renderer>()!=null)
						objChoosen[i].GetComponent<Renderer>().material.shader=matChoosen[i];
					
					objChoosen[i].transform.parent=objParent[i];
					
					BoundRender gzrs=objChoosen[i].GetComponent<BoundRender>();
					Destroy(gzrs);
				}
			}
			objDummy.transform.DetachChildren();
			objDummy.transform.position=Vector3.zero;
			objDummy.transform.localEulerAngles=Vector3.zero;
			
			objDummy.transform.localScale=new Vector3(1,1,1);
			arya.Clear();
		}

        if (Input.GetMouseButtonUp(0) && operMode==Mode.SELECT)
		{
			FixRect(ref drawRect);

			drawSelectRect=false;
			
			GameObject[] objs=	GameObject.FindGameObjectsWithTag("Choose");
			
			objs=AddChildObject(objs);
			
			objChoosen=new GameObject[objs.Length];
			matChoosen=new Shader[objs.Length];
			objParent=new Transform[objs.Length];
			int k=0;
			for(int i=0;i<objs.Length;i++)
			{
				Mesh sharedMesh=null;
				
				Bounds bonds=new Bounds();
				if(objs[i].GetComponent<MeshFilter>()==null || objs[i].GetComponent<MeshRenderer>()==null )
				{
					if(objs[i].GetComponent<SkinnedMeshRenderer>()==null)
					{
						continue;
					}
					else
					{
						sharedMesh=objs[i].GetComponent<SkinnedMeshRenderer>().sharedMesh;
						bonds=objs[i].GetComponent<SkinnedMeshRenderer>().GetComponent<Renderer>().bounds;
					}
				}
				else
				{
					MeshFilter meshfilter=objs[i].GetComponent<MeshFilter>();
					MeshRenderer meshRender= objs[i].GetComponent<MeshRenderer>();
					sharedMesh=meshfilter.sharedMesh;
					bonds=meshRender.GetComponent<Renderer>().bounds;
				}
				Vector3[]  vertices;
				
				vertices=sharedMesh.vertices;
				
				Vector3    vec3=Vector3.zero;
				Vector2    vec2=Vector2.zero;
				for(int j=0;j<vertices.Length;j++)
				{
					
					vec3=GetComponent<Camera>().WorldToScreenPoint( objs[i].transform.TransformPoint(  vertices[j]));
					
					vec2.x=vec3.x;
					vec2.y=vec3.y;
					
					if(drawRect.Contains(vec2))
					{
						if(objs[i].GetComponent<BoundRender>()==null)
						{
							objs[i].AddComponent<BoundRender>();
							objDummy.transform.position+= bonds.center;
							
							operMode=Mode.OPER;
							objChoosen[k]=objs[i];
							
							matChoosen[k]=objs[i].GetComponent<Renderer>().material.shader;
							objChoosen[k].GetComponent<Renderer>().material.shader= Shader.Find("Transparent/Diffuse");
							
							k++;
						}
						break;
					}
				}
			}
			if(k!=0)
			{
				objDummy.transform.position/=k;
				for(int j=0;j<k;j++)
				{
					if(objChoosen[j].GetComponent<MeshCollider>()!=null)
						objChoosen[j].GetComponent<MeshCollider>().enabled=false;
					
					objParent[j]=objChoosen[j].transform.parent;
					objChoosen[j].transform.parent=objDummy.transform;
				}
				if(objDummy.GetComponent<GizmoRender>()==null)
				{
					objDummy.AddComponent<GizmoRender>();
					objDummy.GetComponent<GizmoRender>().eAxis=true;
				}
			}
		}

		if(drawSelectRect && operMode==Mode.SELECT)
		{
			drawRect.height=Input.mousePosition.y-drawRect.y;
			drawRect.width =Input.mousePosition.x-drawRect.x;
		}
		
		if(Input.GetKeyDown( KeyCode.F))
		{
			if(objDummy.GetComponent<GizmoRender>())
			{
			objDummy.GetComponent<GizmoRender>().eAxis=true;
			objDummy.GetComponent<GizmoRender>().eRotate=false;
			objDummy.GetComponent<GizmoRender>().eScal=false;
			}
		}
		if(Input.GetKeyDown( KeyCode.G))
		{
			if(objDummy.GetComponent<GizmoRender>())
			{
			objDummy.GetComponent<GizmoRender>().eAxis=false;
			objDummy.GetComponent<GizmoRender>().eRotate=true;
			objDummy.GetComponent<GizmoRender>().eScal=false;
			}
		}
		
		if(Input.GetKeyDown( KeyCode.H))
		{
			if(objDummy.GetComponent<GizmoRender>())
			{
			objDummy.GetComponent<GizmoRender>().eAxis=false;
			objDummy.GetComponent<GizmoRender>().eRotate=false;
			objDummy.GetComponent<GizmoRender>().eScal=true;
			}
		}
	}

	void FixRect(ref Rect rect)
	{
		if(rect.width<0 )
		{
			rect.x+=rect.width;
			rect.width=-rect.width;
		}
		if(rect.height<0)
		{	
			rect.y+=rect.height;
			rect.height=-rect.height;
		}
	}

	GameObject[] AddChildObject(GameObject[] objs)
	{
		for(int i=0;i<objs.Length;i++)
		{
			arya.Add(objs[i]);
			CountChildren( objs[i].transform);
		}
		
		GameObject[] objret=new GameObject[arya.Count];
		for(int j=0;j<arya.Count;j++)
			objret[j]=(GameObject) arya[j];
		
		return objret;
	}

	void  CountChildren(Transform c)
	{
		foreach(Transform a in c)
		{
			if(a.gameObject.CompareTag("Choose"))
			{
				break;
			}
			else
			{
				arya.Add(a.gameObject);
			}
			CountChildren(a);
		}
	}

	void OnRenderObject()
	{

		if (drawSelectRect) {
			lineMat.SetPass (0);
			GL.Color (new Color (1f, 1f, 1f, 0.2f));
			GL.PushMatrix ();
			GL.LoadOrtho ();
			
			GL.Begin (GL.QUADS);
			
			if (drawRect.width * drawRect.height < 0) {
				GL.Vertex3 (drawRect.x / Screen.width, drawRect.y / Screen.height, 0);
				GL.Vertex3 ((drawRect.x + drawRect.width) / Screen.width, drawRect.y / Screen.height, 0);
				GL.Vertex3 ((drawRect.x + drawRect.width) / Screen.width, (drawRect.y + drawRect.height) / Screen.height, 0);
				GL.Vertex3 (drawRect.x / Screen.width, (drawRect.y + drawRect.height) / Screen.height, 0);
			} else {
				GL.Vertex3 (drawRect.x / Screen.width, drawRect.y / Screen.height, 0);
				GL.Vertex3 (drawRect.x / Screen.width, (drawRect.y + drawRect.height) / Screen.height, 0);
				GL.Vertex3 ((drawRect.x + drawRect.width) / Screen.width, (drawRect.y + drawRect.height) / Screen.height, 0);
				GL.Vertex3 ((drawRect.x + drawRect.width) / Screen.width, drawRect.y / Screen.height, 0);
			}
			GL.End ();
			GL.PopMatrix ();
		}
	}
}