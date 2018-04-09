Shader "Custom/PointerShader"
{
	Properties
	{
		_MainTex ("Texture", 2D) = "white" {}
		_Color ("Color", Color) = (1,1,1,1)
	}
	SubShader
	{
		Tags { "RenderType"="Opaque" }
		LOD 100

		Pass {
			Cull Off
			Zwrite Off
			ColorMask 0
		}

		UsePass "Transparent/Diffuse/FORWARD"
	}
}
