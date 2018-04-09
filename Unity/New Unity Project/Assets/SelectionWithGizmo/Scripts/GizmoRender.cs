using UnityEngine;
using System.Collections;

public class GizmoRender : MonoBehaviour {
	
	public    Material  lineMaterial;
	Vector3   untransRight,untransUp,untransForward,Dir;
	Color     colorRed,colorGreen,colorBlue,colorFan,colorWhite,colorLine,colorWire,selX,selY,selZ,panY,panX,panZ,circleX,circleY,circleZ;
	Plane     planeXZ,planeYZ,planeXY;
	public float     lineSelectedThick=0.04f;
	public float     lineRSelectedThick=0.1f;
	public    float     generalLens=1f;
	enum      MOVETYPE{NONE,X,X2,Y,Y2,Z,Z2,XZ,YZ,XY,RX,RY,RZ,TX,TY,TZ,TX2,TY2,TZ2,TXY,TXZ,TYZ};
    MOVETYPE  beSelectedType;
	Vector3   hitMouseDown,hitMouseMove;
	bool      moveLockVertex;
    Matrix4x4 locklocalToWorldMatrix;
	float     rotaCal;
	Vector3   matScal;
	Matrix4x4 rotationMatrix;
	Vector3   uForward,rForward,fForward;
	
	public    bool      eAxis,eRotate,eScal;
	Vector3   oScal;
	 
	void Awake(){
 
		lineMaterial=new Material(Shader.Find("Particles/Additive"));
		untransRight  =new Vector3(1,0,0);
		untransUp     =new Vector3(0,1,0);
		untransForward=new Vector3(0,0,1);
		Dir=Vector3.zero;
		colorRed=new  Color(1,0,0,1f);
		colorGreen=new Color(0,1,0,1f);
		colorBlue=new Color(0,0,1,1f);
		colorFan=new Color(1,1,0,0.5f);
		colorWhite=new  Color(1,1,1,0.3f);
	    colorWire=new Color(1,1,1,0.05f);
		colorLine=new Color(1,1,0,0.8f);
        selX=colorRed;
		selY=colorGreen;
		selZ=colorBlue;
		panY=colorGreen;
		panX=colorRed;
		panZ=colorBlue;
		circleX=colorRed;
		circleY=colorGreen;
		circleZ=colorBlue;
	    beSelectedType=MOVETYPE.NONE;
		hitMouseDown=hitMouseMove=Vector3.zero;
	    moveLockVertex=false;
	    matScal=new Vector3(1,1,1);
	    rotationMatrix=Matrix4x4.TRS(transform.position,transform.localRotation,matScal);
		locklocalToWorldMatrix=rotationMatrix;
	    eAxis=eRotate=eScal=false;
		oScal=transform.localScale;
	}

	void OnRenderObject(){
		if (beSelectedType == MOVETYPE.NONE) {
			panY = colorGreen;
			panX = colorRed;
			panZ = colorBlue;
			selX = colorRed;
			selY = colorGreen;
			selZ = colorBlue;
			circleX = colorRed;
			circleY = colorGreen;
			circleZ = colorBlue;
			moveLockVertex = false;
	   
			oScal = transform.localScale;
		}
		if (!Input.GetMouseButton (0)) {
			beSelectedType = MOVETYPE.NONE;
			goto Rd;
		}
		
		planeXZ.SetNormalAndPosition (transform.up, transform.position);
		planeXY.SetNormalAndPosition (transform.forward, transform.position);
		planeYZ.SetNormalAndPosition (transform.right, transform.position);		
		
		Ray ray = Camera.main.ScreenPointToRay (Input.mousePosition);
		float enter = 0.0f;
		float rlens = 0.0f;
	   
		planeXZ.Raycast (ray, out enter);
		Vector3 hit = ray.GetPoint (enter);
		
		if (beSelectedType == MOVETYPE.RY) {
			hit = locklocalToWorldMatrix.inverse.MultiplyPoint (hit);
		} else
			hit = rotationMatrix.inverse.MultiplyPoint (hit);
	
		if (eAxis) {
			if (beSelectedType == MOVETYPE.NONE && hit.x > 0f && hit.x <= 0.3f * generalLens && hit.z > 0 && hit.z <= 0.3f * generalLens) {
			
				panY = colorFan;
	
				beSelectedType = MOVETYPE.XZ;
		
				moveLockVertex = true;
				hitMouseDown = hit;
			}
			if (beSelectedType == MOVETYPE.XZ && (Input.GetAxis ("Mouse X") != 0f || Input.GetAxis ("Mouse Y") != 0f)) {
				hitMouseMove = hit - hitMouseDown;
				hitMouseMove.y = 0;
				transform.position += transform.localRotation * hitMouseMove;
			}
			if (beSelectedType == MOVETYPE.NONE && hit.x > 0f && hit.x <= generalLens && Mathf.Abs (hit.z) < lineSelectedThick * generalLens) { //case x
				selX = colorFan;
				beSelectedType = MOVETYPE.X;
				hitMouseDown = hit;
			}
			if (beSelectedType == MOVETYPE.X && (Input.GetAxis ("Mouse X") != 0f || Input.GetAxis ("Mouse Y") != 0f)) {
				hitMouseMove = hit - hitMouseDown;
				hitMouseMove.y = 0;
				hitMouseMove.z = 0;
				transform.position += transform.localRotation * hitMouseMove;
			}
			if (beSelectedType == MOVETYPE.NONE && hit.z > 0f && hit.z <= generalLens && Mathf.Abs (hit.x) < lineSelectedThick * generalLens) {//case  z
				selZ = colorFan;
				beSelectedType = MOVETYPE.Z;
				hitMouseDown = hit;
			}
			if (beSelectedType == MOVETYPE.Z && (Input.GetAxis ("Mouse X") != 0f || Input.GetAxis ("Mouse Y") != 0f)) {
		   	
				hitMouseMove = hit - hitMouseDown;
				hitMouseMove.y = 0;
				hitMouseMove.x = 0;
				transform.position += transform.localRotation * hitMouseMove;
		  
			}
		} //eAxis end
		if (eScal) {
			if (beSelectedType == MOVETYPE.NONE && hit.x > 0f && hit.x <= 0.3f * generalLens && hit.z > 0 && hit.z <= 0.3f * generalLens && hit.x + hit.z - 0.3f * generalLens <= 0f) {
			
				panY = colorFan;
				
	
				beSelectedType = MOVETYPE.TXZ;
		
		
				moveLockVertex = true;
				hitMouseDown = hit;
		
			
			}
			if (beSelectedType == MOVETYPE.TXZ && (Input.GetAxis ("Mouse X") != 0f || Input.GetAxis ("Mouse Y") != 0f)) {
			
		
				hitMouseMove = hit - hitMouseDown;
				hitMouseMove.y = 0;
		   
				float dm = hitMouseMove.x + hitMouseMove.z;
				float dchg = (int)((dm) * 10) / 10f;

				transform.localScale = oScal + new Vector3 (dchg, 0, dchg);
		
		   
			}
		
			if (beSelectedType == MOVETYPE.NONE && hit.x > 0f && hit.x <= generalLens && Mathf.Abs (hit.z) < lineSelectedThick * generalLens) { //case x
				selX = colorFan;
				beSelectedType = MOVETYPE.TX;
				hitMouseDown = hit;
		 
			}
			if (beSelectedType == MOVETYPE.TX && (Input.GetAxis ("Mouse X") != 0f || Input.GetAxis ("Mouse Y") != 0f)) {
		   	
				hitMouseMove = hit - hitMouseDown;
				hitMouseMove.y = 0;
				hitMouseMove.z = 0;
	
				float dm = hitMouseMove.x;
				float dchg = (int)((dm) * 10) / 10f;

				transform.localScale = oScal + new Vector3 (dchg, 0, 0);
		    
			}
			if (beSelectedType == MOVETYPE.NONE && hit.z > 0f && hit.z <= generalLens && Mathf.Abs (hit.x) < lineSelectedThick * generalLens) {//case  z
				selZ = colorFan;
				beSelectedType = MOVETYPE.TZ;
				hitMouseDown = hit;
		
			}
			if (beSelectedType == MOVETYPE.TZ && (Input.GetAxis ("Mouse X") != 0f || Input.GetAxis ("Mouse Y") != 0f)) {
		   	
				hitMouseMove = hit - hitMouseDown;
				hitMouseMove.y = 0;
				hitMouseMove.x = 0;
	
				float dm = hitMouseMove.z;
				float dchg = (int)((dm) * 10) / 10f;

				transform.localScale = oScal + new Vector3 (0, 0, dchg);
		  
			}//eScal end
		}
		rlens = Mathf.Sqrt (hit.x * hit.x + hit.z * hit.z);
		
		if (eRotate) {
			if (Vector3.Dot (hit, uForward) >= 0f)
			if (beSelectedType == MOVETYPE.NONE && rlens > (1 - lineRSelectedThick) * generalLens && rlens < (1 + lineRSelectedThick) * generalLens) {
				circleY = colorFan;
				beSelectedType = MOVETYPE.RY;
				moveLockVertex = true;
				hitMouseDown = hit;
		
				locklocalToWorldMatrix = rotationMatrix;
	
				rotaCal = 0.0f;
			}
			if (beSelectedType == MOVETYPE.RY) {  //rotate y
				hitMouseMove = hit;

				float angledown	= Vector3.Dot (hitMouseDown.normalized, untransRight); 
				float anglemove = Vector3.Dot (hitMouseMove.normalized, untransRight); 
				float angledeta = Vector3.Dot (hitMouseDown.normalized, hitMouseMove.normalized);
		
				angledown = SnapDot (angledown, hitMouseDown.x, hitMouseDown.z);	
			
				anglemove = SnapDot (anglemove, hitMouseMove.x, hitMouseMove.z);	
		
				angledeta = Mathf.Acos (angledeta);
		   
				if (angledown == anglemove)
					goto Rd;
		   	
				float judge = angledown + Mathf.PI;
			
				if (judge > 2 * Mathf.PI) {
					if (anglemove >= 0f && anglemove <= judge - 2 * Mathf.PI || anglemove > angledown) {     
					} else
						angledeta = -angledeta;
				} else {
					if (anglemove >= angledown && anglemove <= judge) {
					} else
						angledeta = -angledeta;
				}
				rotaCal = angledeta - rotaCal;
		 
				DrawCam (colorWhite, angledown, angledeta, MOVETYPE.Y);
		   
				transform.RotateAround (transform.up, -rotaCal);
				//	transform.Rotate(transform.up,-rotaCal);
	       
				rotaCal = angledeta;  
			}
		}//eRotate end

		enter = 0.0f;
		planeXY.Raycast (ray, out enter);
		hit = ray.GetPoint (enter);
		
		if (beSelectedType == MOVETYPE.RZ) {
			hit = locklocalToWorldMatrix.inverse.MultiplyPoint (hit);
		} else
			hit = rotationMatrix.inverse.MultiplyPoint (hit);
		
		if (eAxis) {
			if (beSelectedType == MOVETYPE.NONE && hit.x > 0f && hit.x <= 0.3f * generalLens && hit.y > 0 && hit.y <= 0.3f * generalLens) {
				panZ = colorFan;
				beSelectedType = MOVETYPE.XY;
				hitMouseDown = hit;
			}
			if (beSelectedType == MOVETYPE.XY) {
				hitMouseMove = hit - hitMouseDown;
		
				hitMouseMove.z = 0;
				transform.position += transform.localRotation * hitMouseMove;
			}
			if (beSelectedType == MOVETYPE.NONE && hit.x > 0f && hit.x <= generalLens && Mathf.Abs (hit.y) < lineSelectedThick * generalLens) { //case x
				selX = colorFan;
				beSelectedType = MOVETYPE.X2;
	       
				hitMouseDown = hit;
			}
			if (beSelectedType == MOVETYPE.X2 && (Input.GetAxis ("Mouse X") != 0f || Input.GetAxis ("Mouse Y") != 0f)) {
				hitMouseMove = hit - hitMouseDown;
				hitMouseMove.y = 0;
				hitMouseMove.z = 0;
				transform.position += transform.localRotation * hitMouseMove;
			}
			if (beSelectedType == MOVETYPE.NONE && hit.y > 0f && hit.y <= generalLens && Mathf.Abs (hit.x) < lineSelectedThick * generalLens) { //case y
				selY = colorFan;
				beSelectedType = MOVETYPE.Y;
				hitMouseDown = hit;
			}
			if (beSelectedType == MOVETYPE.Y && (Input.GetAxis ("Mouse X") != 0f || Input.GetAxis ("Mouse Y") != 0f)) {
				hitMouseMove = hit - hitMouseDown;
				hitMouseMove.z = 0;
				hitMouseMove.x = 0;
				transform.position += transform.localRotation * hitMouseMove;
			}
		}//eAxis end
		if (eScal) {
			if (beSelectedType == MOVETYPE.NONE && hit.x > 0f && hit.x <= 0.3f * generalLens && hit.y > 0 && hit.y <= 0.3f * generalLens && hit.x + hit.y - 0.3f * generalLens <= 0f) {
				panZ = colorFan;
				beSelectedType = MOVETYPE.TXY;
				hitMouseDown = hit;
			}
			if (beSelectedType == MOVETYPE.TXY) {
				hitMouseMove = hit - hitMouseDown;
				hitMouseMove.z = 0;
		
				float dm = hitMouseMove.x + hitMouseMove.y;
				float dchg = (int)((dm) * 10) / 10f;

				transform.localScale = oScal + new Vector3 (dchg, dchg, 0f);
			}
			if (beSelectedType == MOVETYPE.NONE && hit.x > 0f && hit.x <= generalLens && Mathf.Abs (hit.y) < lineSelectedThick * generalLens) { //case x
				selX = colorFan;
				beSelectedType = MOVETYPE.TX2;
				moveLockVertex = true;
				hitMouseDown = hit;
			}
			if (beSelectedType == MOVETYPE.TX2 && (Input.GetAxis ("Mouse X") != 0f || Input.GetAxis ("Mouse Y") != 0f)) {
				hitMouseMove = hit - hitMouseDown;
				hitMouseMove.y = 0;
				hitMouseMove.z = 0;
		
				float dm = hitMouseMove.x;
				float dchg = (int)((dm) * 10) / 10f;

				transform.localScale = oScal + new Vector3 (dchg, 0, 0);
			}
			if (beSelectedType == MOVETYPE.NONE && hit.y > 0f && hit.y <= generalLens && Mathf.Abs (hit.x) < lineSelectedThick * generalLens) { //case y
				selY = colorFan;
				beSelectedType = MOVETYPE.TY;
				hitMouseDown = hit;
			}
			if (beSelectedType == MOVETYPE.TY && (Input.GetAxis ("Mouse X") != 0f || Input.GetAxis ("Mouse Y") != 0f)) {
				hitMouseMove = hit - hitMouseDown;
				hitMouseMove.z = 0;
				hitMouseMove.x = 0;
	
				float dm = hitMouseMove.y;
				float dchg = (int)((dm) * 10) / 10f;

				transform.localScale = oScal + new Vector3 (0, dchg, 0);
			}
		}//eScal end
		rlens = Mathf.Sqrt (hit.x * hit.x + hit.y * hit.y);
		
		if (eRotate) {
			if (Vector3.Dot (hit, fForward) >= 0f)
			if (beSelectedType == MOVETYPE.NONE && rlens > (1 - lineRSelectedThick) * generalLens && rlens < (1 + lineRSelectedThick) * generalLens) {
				circleZ = colorFan;
				beSelectedType = MOVETYPE.RZ;
				moveLockVertex = true;
				hitMouseDown = hit;	
				locklocalToWorldMatrix = rotationMatrix;
				rotaCal = 0.0f;
			}
			if (beSelectedType == MOVETYPE.RZ) {
				hitMouseMove = hit;

				float angledown	= Vector3.Dot (hitMouseDown.normalized, untransUp); 
				float anglemove = Vector3.Dot (hitMouseMove.normalized, untransUp); 
				float angledeta = Vector3.Dot (hitMouseDown.normalized, hitMouseMove.normalized);
		
				angledown = SnapDot (angledown, hitMouseDown.y, hitMouseDown.x);	
				anglemove = SnapDot (anglemove, hitMouseMove.y, hitMouseMove.x);	
				angledeta = Mathf.Acos (angledeta);
		   
				if (angledown == anglemove)
					goto Rd;
		   	
				float judge = angledown + Mathf.PI;
			
				if (judge > 2 * Mathf.PI) {
					if (anglemove >= 0f && anglemove <= judge - 2 * Mathf.PI || anglemove > angledown) {     
				
					} else
						angledeta = -angledeta;
				} else {
					if (anglemove >= angledown && anglemove <= judge) {
					} else
						angledeta = -angledeta;
				}
				rotaCal = angledeta - rotaCal;
		   
				DrawCam (colorWhite, angledown, angledeta, MOVETYPE.Z);
		
				transform.RotateAround (transform.forward, -rotaCal);
				//   transform.Rotate(transform.forward,-rotaCal);
				rotaCal = angledeta;  
			}
		}//eRotate end
		enter = 0.0f;
		planeYZ.Raycast (ray, out enter);
		hit = ray.GetPoint (enter);
		
		if (beSelectedType == MOVETYPE.RX) {
			hit = locklocalToWorldMatrix.inverse.MultiplyPoint (hit);
		} else
			hit = rotationMatrix.inverse.MultiplyPoint (hit);

		if (eAxis) {
			if (beSelectedType == MOVETYPE.NONE && hit.z > 0 && hit.z <= 0.3f * generalLens && hit.y > 0 && hit.y <= 0.3f * generalLens) {
				panX = colorFan;
				beSelectedType = MOVETYPE.YZ;
				hitMouseDown = hit;
			}
			if (beSelectedType == MOVETYPE.YZ) {
				hitMouseMove = hit - hitMouseDown;
				hitMouseMove.x = 0;
				transform.position += transform.localRotation * hitMouseMove;
			}
			if (beSelectedType == MOVETYPE.NONE && hit.z > 0f && hit.z <= generalLens && Mathf.Abs (hit.y) < lineSelectedThick * generalLens) {//case  z
				selZ = colorFan;
				beSelectedType = MOVETYPE.Z2;
				hitMouseDown = hit;
			}
			if (beSelectedType == MOVETYPE.Z2 && (Input.GetAxis ("Mouse X") != 0f || Input.GetAxis ("Mouse Y") != 0f)) {
				hitMouseMove = hit - hitMouseDown;
				hitMouseMove.y = 0;
				hitMouseMove.x = 0;
				transform.position += transform.localRotation * hitMouseMove;
		  
			}
			if (beSelectedType == MOVETYPE.NONE && hit.y > 0f && hit.y <= generalLens && Mathf.Abs (hit.z) < lineSelectedThick * generalLens) { //case y
				selY = colorFan;
				beSelectedType = MOVETYPE.Y2;
				hitMouseDown = hit;
			}
			if (beSelectedType == MOVETYPE.Y2 && (Input.GetAxis ("Mouse X") != 0f || Input.GetAxis ("Mouse Y") != 0f)) {
				hitMouseMove = hit - hitMouseDown;
				hitMouseMove.z = 0;
				hitMouseMove.x = 0;
				transform.position += transform.localRotation * hitMouseMove;
			}
		}//eAxis end
		if (eScal) {
			if (beSelectedType == MOVETYPE.NONE && hit.z > 0 && hit.z <= 0.3f * generalLens && hit.y > 0 && hit.y <= 0.3f * generalLens && hit.y + hit.z - 0.3f * generalLens <= 0f) {
				panX = colorFan;
				beSelectedType = MOVETYPE.TYZ;
				hitMouseDown = hit;
			}
			if (beSelectedType == MOVETYPE.TYZ) {
				hitMouseMove = hit - hitMouseDown;
			
				hitMouseMove.x = 0;
	
				float dm = hitMouseMove.z + hitMouseMove.y;
				float dchg = (int)((dm) * 10) / 10f;

				transform.localScale = oScal + new Vector3 (0, dchg, dchg);
			}
			if (beSelectedType == MOVETYPE.NONE && hit.z > 0f && hit.z <= generalLens && Mathf.Abs (hit.y) < lineSelectedThick * generalLens) {//case  z
				selZ = colorFan;
				beSelectedType = MOVETYPE.TZ2;
				hitMouseDown = hit;
			}
			if (beSelectedType == MOVETYPE.TZ2 && (Input.GetAxis ("Mouse X") != 0f || Input.GetAxis ("Mouse Y") != 0f)) {
		   	
				hitMouseMove = hit - hitMouseDown;
				hitMouseMove.y = 0;
				hitMouseMove.x = 0;
			
				float dm = hitMouseMove.z;
				float dchg = (int)((dm) * 10) / 10f;

				transform.localScale = oScal + new Vector3 (0, 0, dchg);
		  
			}
			if (beSelectedType == MOVETYPE.NONE && hit.y > 0f && hit.y <= generalLens && Mathf.Abs (hit.z) < lineSelectedThick * generalLens) { //case y
				selY = colorFan;
				beSelectedType = MOVETYPE.TY2;
				hitMouseDown = hit;
			}
			if (beSelectedType == MOVETYPE.TY2 && (Input.GetAxis ("Mouse X") != 0f || Input.GetAxis ("Mouse Y") != 0f)) {
		   	
				hitMouseMove = hit - hitMouseDown;
				hitMouseMove.z = 0;
				hitMouseMove.x = 0;
		
				float dm = hitMouseMove.y;
				float dchg = (int)((dm) * 10) / 10f;

				transform.localScale = oScal + new Vector3 (0, dchg, 0);
			}
		}//eScal end
		rlens = Mathf.Sqrt (hit.z * hit.z + hit.y * hit.y);
		if (eRotate) {
			if (Vector3.Dot (hit, rForward) >= 0f)
			if (beSelectedType == MOVETYPE.NONE && rlens > (1 - lineRSelectedThick) * generalLens && rlens < (1 + lineRSelectedThick) * generalLens) {
				circleX = colorFan;
				beSelectedType = MOVETYPE.RX;
				moveLockVertex = true;
				hitMouseDown = hit;	
				locklocalToWorldMatrix = rotationMatrix;
				rotaCal = 0.0f;
			}
			if (beSelectedType == MOVETYPE.RX) {
				hitMouseMove = hit;

				float angledown	= Vector3.Dot (hitMouseDown.normalized, untransForward); 
				float anglemove = Vector3.Dot (hitMouseMove.normalized, untransForward); 
				float angledeta = Vector3.Dot (hitMouseDown.normalized, hitMouseMove.normalized);
		
				angledown = SnapDot (angledown, hitMouseDown.z, hitMouseDown.y);	
			
				anglemove = SnapDot (anglemove, hitMouseMove.z, hitMouseMove.y);	
	
				angledeta = Mathf.Acos (angledeta);
		   
				if (angledown == anglemove)
					goto Rd;
		   	
				float judge = angledown + Mathf.PI;
			
				if (judge > 2 * Mathf.PI) {
					if (anglemove >= 0f && anglemove <= judge - 2 * Mathf.PI || anglemove > angledown) {     
					} else
						angledeta = -angledeta;
				} else {
					if (anglemove >= angledown && anglemove <= judge) {
					} else
						angledeta = -angledeta;
				}
				rotaCal = angledeta - rotaCal;
		   
				DrawCam (colorWhite, angledown, angledeta, MOVETYPE.X);
		
				transform.RotateAround (transform.right, -rotaCal);
				transform.Rotate (transform.right, -rotaCal);
		  
				rotaCal = angledeta;  
			}
		}//eRotate end

		Rd:
		RenderGizmos ();
	}

	void RenderGizmos(){
		Dir = Camera.main.transform.position - transform.position;
	 
		float d = Dir.magnitude;
		generalLens = -2.0f * Mathf.Tan (0.5f * Camera.main.fieldOfView) * d / 50;
		Dir.Normalize ();
	
		GL.PushMatrix (); 
		rotationMatrix = transform.localToWorldMatrix;
	
		rotationMatrix = Matrix4x4.TRS (transform.position, transform.localRotation, matScal);
		GL.MultMatrix (rotationMatrix); 
		lineMaterial.SetPass (0);
		
		if (eRotate) {
			//project Dir on planeXZ
			Vector3 camdir = transform.InverseTransformPoint (Camera.main.transform.position);  
		
			Vector3 prjleft = Vector3.Cross (untransRight, camdir);
			prjleft.Normalize ();
			prjleft *= generalLens;
			Vector3 prjfwrd = Vector3.Cross (prjleft, untransRight);
			prjfwrd.Normalize ();	
			prjfwrd *= generalLens;
			DrawCircleHalf (circleX, prjleft, prjfwrd);
			rForward = prjfwrd;	
 
			prjleft = Vector3.Cross (untransUp, camdir);
			prjleft.Normalize ();
			prjleft *= generalLens;
			prjfwrd = Vector3.Cross (prjleft, untransUp);
			prjfwrd.Normalize ();
			prjfwrd *= generalLens;
			DrawCircleHalf (circleY, prjleft, prjfwrd);
			uForward = prjfwrd;	

			prjleft = Vector3.Cross (untransForward, camdir);
			prjleft.Normalize ();
			prjleft *= generalLens;
			prjfwrd = Vector3.Cross (prjleft, untransForward);
			prjfwrd.Normalize ();
			prjfwrd *= generalLens;
			DrawCircleHalf (circleZ, prjleft, prjfwrd);
			fForward = prjfwrd;
		}
		
		if (eAxis) {	
			DrawAxis (untransRight * generalLens, untransUp, untransForward, 0.04f * generalLens, 0.9f, selX);	
			DrawAxis (untransUp * generalLens, untransRight, untransForward, 0.04f * generalLens, 0.9f, selY);
			DrawAxis (untransForward * generalLens, untransRight, untransUp, 0.04f * generalLens, 0.9f, selZ);
		
			DrawQuad (0.3f * generalLens, false, untransForward, untransUp, panX);
			DrawQuad (0.3f * generalLens, false, untransRight, untransForward, panY);
			DrawQuad (0.3f * generalLens, false, untransRight, untransUp, panZ);
		}

		if (eScal) {
			DrawAxis (untransRight * generalLens, untransUp, untransForward, 0.04f * generalLens, 0.9f, selX);	
			DrawAxis (untransUp * generalLens, untransRight, untransForward, 0.04f * generalLens, 0.9f, selY);
			DrawAxis (untransForward * generalLens, untransRight, untransUp, 0.04f * generalLens, 0.9f, selZ);
			DrawTri (0.3f * generalLens, false, untransForward, untransUp, panX);
			DrawTri (0.3f * generalLens, false, untransRight, untransForward, panY);
			DrawTri (0.3f * generalLens, false, untransRight, untransUp, panZ);
		}

		GL.PopMatrix ();
	
		//GL.PushMatrix();
		//render wireframe
		//lineMaterial.SetPass(0);
		//GL.MultMatrix(transform.localToWorldMatrix);
		
		//DrawBondBox();
		
		//GL.PopMatrix();
	}

	void DrawCam(Color col, float sang,float eng ,MOVETYPE dtype){
		GL.PushMatrix (); 
		rotationMatrix = locklocalToWorldMatrix;

		GL.MultMatrix (rotationMatrix); 
		lineMaterial.SetPass (0);
		GL.Color (col);
	
		GL.Begin (GL.TRIANGLES);
		
		float ang;
	    
		switch (dtype) {
		case MOVETYPE.X:
			for (int i = 0; i < 20; i++) {
				ang = sang + (eng) * i / 20;
		
				GL.Vertex3 (0, 0, 0);
				GL.Vertex3 (0, Mathf.Sin (ang) * generalLens, Mathf.Cos (ang) * generalLens);
				ang = sang + (eng) * (i + 1) / 20;
				GL.Vertex3 (0, Mathf.Sin (ang) * generalLens, Mathf.Cos (ang) * generalLens);
			}  
			break;
			
		case MOVETYPE.Y:
			for (int i = 0; i < 20; i++) {
				ang = sang + (eng) * i / 20;
		
				GL.Vertex3 (0, 0, 0);
				GL.Vertex3 (Mathf.Cos (ang) * generalLens, 0, Mathf.Sin (ang) * generalLens);
				ang = sang + (eng) * (i + 1) / 20;
				GL.Vertex3 (Mathf.Cos (ang) * generalLens, 0, Mathf.Sin (ang) * generalLens);
			}  
			break;

		case MOVETYPE.Z:
			for (int i = 0; i < 20; i++) {
				ang = sang + (eng) * i / 20;
		
				GL.Vertex3 (0, 0, 0);
				GL.Vertex3 (Mathf.Sin (ang) * generalLens, Mathf.Cos (ang) * generalLens, 0);
				ang = sang + (eng) * (i + 1) / 20;
				GL.Vertex3 (Mathf.Sin (ang) * generalLens, Mathf.Cos (ang) * generalLens, 0);
			}  
			break;
		}
		
		GL.End ();
		GL.PopMatrix ();
	}
	
	void DrawCircle(Color col, Vector3 vtx, Vector3 vty){
		GL.Color (col);
		GL.Begin (GL.LINES);
		for (int i = 0; i < 100; i++) {
			Vector3 vt;
			vt = vtx * Mathf.Cos ((2 * Mathf.PI / 100) * i);
			vt += vty * Mathf.Sin ((2 * Mathf.PI / 100) * i);

			GL.Vertex3 (vt.x, vt.y, vt.z);
			vt = vtx * Mathf.Cos ((2 * Mathf.PI / 100) * (i + 1));
			vt += vty * Mathf.Sin ((2 * Mathf.PI / 100) * (i + 1));

			GL.Vertex3 (vt.x, vt.y, vt.z);
		}
		GL.End ();
	}

	float  SnapDot(float dot,float x,float y){
		float thd = 0.0f;  
	
		if (dot >= 0) {
			if (y > 0)
				thd = Mathf.Acos (dot);
			else
				thd = 2 * Mathf.PI - Mathf.Acos (dot);
		} else {
			if (y > 0)
				thd = Mathf.Acos (dot);
			else
				thd = 2 * Mathf.PI - Mathf.Acos (dot);
		}
		return thd;
	}
	
	void DrawCircleHalf(Color col, Vector3 vtx, Vector3 vty){
		GL.Color (col);
		GL.Begin (GL.LINES);
		for (int i = 0; i < 100; i++) {
			if (true) {
				Vector3 vt;
				vt = vtx * Mathf.Cos ((Mathf.PI / 100) * i);
				vt += vty * Mathf.Sin ((Mathf.PI / 100) * i);
				GL.Vertex3 (vt.x, vt.y, vt.z);
				vt = vtx * Mathf.Cos ((Mathf.PI / 100) * (i + 1));
				vt += vty * Mathf.Sin ((Mathf.PI / 100) * (i + 1));
			
				GL.Vertex3 (vt.x, vt.y, vt.z);
			}
		}
		GL.End ();
	}

	void DrawAxis(Vector3 axis, Vector3 vtx,Vector3 vty, float fct,float fct2,Color col){
		GL.Color (col);
		GL.Begin (GL.LINES);
		GL.Vertex3 (0, 0, 0);
		GL.Vertex (axis);
		GL.End ();

		GL.Begin (GL.TRIANGLES);
		for (int i = 0; i <= 30; i++) {
			Vector3 pt;
			pt = vtx * Mathf.Cos (((2 * Mathf.PI) / 30.0f) * i) * fct;
			pt += vty * Mathf.Sin (((2 * Mathf.PI) / 30.0f) * i) * fct;
			pt += axis * fct2;
	
			GL.Vertex (pt);
			pt = vtx * Mathf.Cos (((2 * Mathf.PI) / 30.0f) * (i + 1)) * fct;
			pt += vty * Mathf.Sin (((2 * Mathf.PI) / 30.0f) * (i + 1)) * fct;
			pt += axis * fct2;
	
			GL.Vertex (pt);
			GL.Vertex (axis);
		}
		GL.End ();
	}

	void DrawAxisS(Vector3 axis,Color col){
		GL.Color (col);
		GL.Begin (GL.LINES);
		GL.Vertex3 (0, 0, 0);
		GL.Vertex (axis);
		GL.End ();
	}	

	void DrawFan(Vector3 vtx,Vector3 vty,float ng){
		GL.Color (colorFan);
		for (int i = 0; i <= 50; i++) {
			GL.Begin (GL.TRIANGLES);
			Vector3 vt;
			vt = vtx * Mathf.Cos (((ng) / 50) * i);
			vt += vty * Mathf.Sin (((ng) / 50) * i);
			GL.Vertex (vt);
			
			vt = vtx * Mathf.Cos (((ng) / 50) * (i + 1));
			vt += vty * Mathf.Sin (((ng) / 50) * (i + 1));
			GL.Vertex (vt);
			GL.Vertex (Vector3.zero);
			GL.End ();
		}
	}

	void DrawQuad(float size, bool bSelected, Vector3 axisU,Vector3 axisV,Color col){
		col.a = 0.3f;
		Vector3[] pts = new Vector3[4];
		pts [0] = Vector3.zero;
		pts [1] = (axisU * size);
		pts [2] = (axisU + axisV) * size;
		pts [3] = (axisV * size);
    
		if (!bSelected)
			GL.Color (col);
		else
			GL.Color (colorWhite);
		GL.Begin (GL.QUADS);
		GL.Vertex (pts [0]);
		GL.Vertex (pts [1]);
		GL.Vertex (pts [2]);
		GL.Vertex (pts [3]);
		GL.End ();
		col.a = 1f;
		if (!bSelected)
			GL.Color (col);
		else
			GL.Color (colorWhite);
		GL.Begin (GL.LINES);
		GL.Vertex (pts [0]);
		GL.Vertex (pts [1]);
		GL.Vertex (pts [1]);
		GL.Vertex (pts [2]);
		GL.Vertex (pts [2]);
		GL.Vertex (pts [3]);
		GL.Vertex (pts [3]);
		GL.Vertex (pts [0]);
		GL.End ();
	}

	void DrawTri( float size, bool bSelected, Vector3 axisU, Vector3 axisV,Color col){
		col.a = 0.3f;
		Vector3[] pts = new Vector3[3];
		pts [0] = Vector3.zero;

		pts [1] = (axisU);
		pts [2] = (axisV);

		pts [1] *= size;
		pts [2] *= size;
	
		if (!bSelected)
			GL.Color (col);
		else
			GL.Color (colorWhite);
		GL.Begin (GL.TRIANGLES);
		GL.Vertex (pts [0]);
		GL.Vertex (pts [1]);
		GL.Vertex (pts [2]);
		GL.End ();
		col.a = 1f;
		if (!bSelected)
			GL.Color (col);
		else
			GL.Color (colorWhite);
		GL.Begin (GL.LINES);
		GL.Vertex (pts [0]);
		GL.Vertex (pts [1]);
		GL.Vertex (pts [1]);
		GL.Vertex (pts [2]);
		GL.Vertex (pts [2]);
		GL.Vertex (pts [0]);
		GL.End ();
	}

	void DrawCube(float size, bool bSelected,Vector3 center,Vector3 axisU,Vector3 axisV, Color col,MOVETYPE move){
		Vector3[] pts = new Vector3[8];
		center.x -= size / 2;
		center.y -= size / 2;
		center.z -= size / 2;
		pts [0] = center;
		pts [1] = center + (axisU * size);
		pts [2] = center + (axisU + axisV) * size;
		pts [3] = center + (axisV * size);
    
		pts [4] = pts [0];
		if (move == MOVETYPE.X)
			pts [4].x += size;
		if (move == MOVETYPE.Y)
			pts [4].y += size;
		if (move == MOVETYPE.Z)
			pts [4].z += size;
		
		pts [5] = pts [1];
		if (move == MOVETYPE.X)
			pts [5].x += size;
		if (move == MOVETYPE.Y)
			pts [5].y += size;
		if (move == MOVETYPE.Z)
			pts [5].z += size;
		pts [6] = pts [2];
		if (move == MOVETYPE.X)
			pts [6].x += size;
		if (move == MOVETYPE.Y)
			pts [6].y += size;
		if (move == MOVETYPE.Z)
			pts [6].z += size;
		pts [7] = pts [3];
		if (move == MOVETYPE.X)
			pts [7].x += size;
		if (move == MOVETYPE.Y)
			pts [7].y += size;
		if (move == MOVETYPE.Z)
			pts [7].z += size;
		
		if (!bSelected)
			GL.Color (col);
		else
			GL.Color (colorWhite);
		GL.Begin (GL.QUADS);
		GL.Vertex (pts [0]);
		GL.Vertex (pts [1]);
		GL.Vertex (pts [2]);
		GL.Vertex (pts [3]);
     
		GL.Vertex (pts [4]);
		GL.Vertex (pts [5]);
		GL.Vertex (pts [6]);
		GL.Vertex (pts [7]);
	
		GL.Vertex (pts [1]);
		GL.Vertex (pts [5]);
		GL.Vertex (pts [6]);
		GL.Vertex (pts [2]);
     
		GL.Vertex (pts [0]);
		GL.Vertex (pts [4]);
		GL.Vertex (pts [7]);
		GL.Vertex (pts [3]);
		
		GL.Vertex (pts [0]);
		GL.Vertex (pts [1]);
		GL.Vertex (pts [5]);
		GL.Vertex (pts [4]);
     
		GL.Vertex (pts [3]);
		GL.Vertex (pts [2]);
		GL.Vertex (pts [6]);
		GL.Vertex (pts [7]);
		
		GL.End ();
	}
}