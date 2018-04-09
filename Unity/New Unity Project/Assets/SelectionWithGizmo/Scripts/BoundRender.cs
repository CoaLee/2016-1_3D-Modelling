using UnityEngine;
using System.Collections;

public class BoundRender : MonoBehaviour {

   public Material lineMaterial;
   Bounds   bonds;

   void Start()
	{		 
		lineMaterial=new Material(Shader.Find("Particles/Additive"));
		bonds=new Bounds();
	}

	void OnRenderObject()
	{
		GL.Color(new Color(1f,1f,1f,0.8f));
		GL.PushMatrix();

		lineMaterial.SetPass(0);
		
		DrawBondBox();
		
		GL.PopMatrix();
	}

	void DrawBondBox()
	{
		bonds = transform.GetComponent<Renderer> ().bounds;

		float l = 6f;
		GL.Begin (GL.LINES);
			
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y + bonds.size.y / 2, bonds.center.z + bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2 - bonds.size.x / l, bonds.center.y + bonds.size.y / 2, bonds.center.z + bonds.size.z / 2);
			
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y + bonds.size.y / 2, bonds.center.z + bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y + bonds.size.y / 2 - bonds.size.y / l, bonds.center.z + bonds.size.z / 2);
			
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y + bonds.size.y / 2, bonds.center.z + bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y + bonds.size.y / 2, bonds.center.z + bonds.size.z / 2 - bonds.size.z / l);
			
			
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y + bonds.size.y / 2, bonds.center.z + bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2 + bonds.size.x / l, bonds.center.y + bonds.size.y / 2, bonds.center.z + bonds.size.z / 2);
			
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y + bonds.size.y / 2, bonds.center.z + bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y + bonds.size.y / 2 - bonds.size.y / l, bonds.center.z + bonds.size.z / 2);
			
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y + bonds.size.y / 2, bonds.center.z + bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y + bonds.size.y / 2, bonds.center.z + bonds.size.z / 2 - bonds.size.z / l);
			
			
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y - bonds.size.y / 2, bonds.center.z + bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2 - bonds.size.x / l, bonds.center.y - bonds.size.y / 2, bonds.center.z + bonds.size.z / 2);
			
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y - bonds.size.y / 2, bonds.center.z + bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y - bonds.size.y / 2 + bonds.size.y / l, bonds.center.z + bonds.size.z / 2);
			
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y - bonds.size.y / 2, bonds.center.z + bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y - bonds.size.y / 2, bonds.center.z + bonds.size.z / 2 - bonds.size.z / l);
			
			
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y - bonds.size.y / 2, bonds.center.z + bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2 + bonds.size.x / l, bonds.center.y - bonds.size.y / 2, bonds.center.z + bonds.size.z / 2);
			
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y - bonds.size.y / 2, bonds.center.z + bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y - bonds.size.y / 2 + bonds.size.y / l, bonds.center.z + bonds.size.z / 2);
			
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y - bonds.size.y / 2, bonds.center.z + bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y - bonds.size.y / 2, bonds.center.z + bonds.size.z / 2 - bonds.size.z / l);
			
			
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y - bonds.size.y / 2, bonds.center.z - bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2 + bonds.size.x / l, bonds.center.y - bonds.size.y / 2, bonds.center.z - bonds.size.z / 2);
			
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y - bonds.size.y / 2, bonds.center.z - bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y - bonds.size.y / 2 + bonds.size.y / l, bonds.center.z - bonds.size.z / 2);
			
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y - bonds.size.y / 2, bonds.center.z - bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y - bonds.size.y / 2, bonds.center.z - bonds.size.z / 2 + bonds.size.z / l);
			
			
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y + bonds.size.y / 2, bonds.center.z - bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2 + bonds.size.x / l, bonds.center.y + bonds.size.y / 2, bonds.center.z - bonds.size.z / 2);
			
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y + bonds.size.y / 2, bonds.center.z - bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y + bonds.size.y / 2 - bonds.size.y / l, bonds.center.z - bonds.size.z / 2);
			
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y + bonds.size.y / 2, bonds.center.z - bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x - bonds.size.x / 2, bonds.center.y + bonds.size.y / 2, bonds.center.z - bonds.size.z / 2 + bonds.size.z / l);
			
			
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y + bonds.size.y / 2, bonds.center.z - bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2 - bonds.size.x / l, bonds.center.y + bonds.size.y / 2, bonds.center.z - bonds.size.z / 2);
			
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y + bonds.size.y / 2, bonds.center.z - bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y + bonds.size.y / 2 - bonds.size.y / l, bonds.center.z - bonds.size.z / 2);
			
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y + bonds.size.y / 2, bonds.center.z - bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y + bonds.size.y / 2, bonds.center.z - bonds.size.z / 2 + bonds.size.z / l);
			
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y - bonds.size.y / 2, bonds.center.z - bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2 - bonds.size.x / l, bonds.center.y - bonds.size.y / 2, bonds.center.z - bonds.size.z / 2);
			
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y - bonds.size.y / 2, bonds.center.z - bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y - bonds.size.y / 2 + bonds.size.y / l, bonds.center.z - bonds.size.z / 2);
			
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y - bonds.size.y / 2, bonds.center.z - bonds.size.z / 2);
		GL.Vertex3 (bonds.center.x + bonds.size.x / 2, bonds.center.y - bonds.size.y / 2, bonds.center.z - bonds.size.z / 2 + bonds.size.z / l);
		GL.End ();
	}
}
