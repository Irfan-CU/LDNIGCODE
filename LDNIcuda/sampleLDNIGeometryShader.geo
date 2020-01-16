/*
 *  Copyright (C) 2014, Geometric Design and Manufacturing Lab in THE CHINESE UNIVERSITY OF HONG KONG
 *  All rights reserved.
 *   
 *		 http://ldnibasedsolidmodeling.sourceforge.net/
 *  
 *   
 *  Redistribution and use in source and binary forms, with or without modification, 
 *  are permitted provided that the following conditions are met:
 *  
 *  1. Redistributions of source code must retain the above copyright notice, 
 *     this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice, 
 *     this list of conditions and the following disclaimer in the documentation 
 *	   and/or other materials provided with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 *   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 *   IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 *   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 *   OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 *   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 *   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 *   OF SUCH DAMAGE.
 */
 /*
 preliminary formula for material distribution (gl_BackColorIn[0].x)+ (gl_PositionIn[0].x) + (gl_FrontColorIn[0].x)
 */

#version 120 
#extension GL_EXT_geometry_shader4: enable

uniform vec3 Cent;
varying out vec4 color;


vec4 CalPlaneEq(vec3 P0, vec3 P1, vec3 P2)
{
	vec4 result;

	result.xyz = P1.xyz+Cent;
	P0.xyz=P0.xyz+Cent;
	P2.xyz=P2.xyz+Cent;


	if (result.x > P0.x || result.x>P2.x)
	{
	  if (result.x > P0.x) result.x = P0.x;
	  if (result.x > P2.x) result.x = P2.x;
	}
	

	if (result.y>P0.y || result.y>P2.y)
	{
	  if (result.y > P0.y) result.y = P0.y;
	  if (result.y > P2.y) result.y = P2.y;
	}

	if (result.z>P0.z || result.z>P2.z)
	{
	  if (result.z > P0.z) result.z = P0.z;
	  if (result.z > P2.z) result.z = P2.z;
	}

    P0.xyz=P0.xyz+Cent;
	P2.xyz=P2.xyz+Cent;

    result.w = result.z;
	result.z = P0.x * ( P1.y - P2.y ) + P1.x * ( P2.y - P0.y ) + P2.x * ( P0.y - P1.y );
	
	result.x = (result.x)*0.06 + 0.2;
	result.y = (result.x)*0.06 + 0.2;
	result.w = (result.w)*0.06 + 0.2;

	float  tt = length(result.xyz);
	if (tt < 0.00000001) return vec4(0.0,0.0,0.0,0.0);

	//result = result/tt;

	return result;
}

void main( void )
{
	color = CalPlaneEq(gl_BackColorIn[0].xyz, gl_PositionIn[0].xyz, gl_FrontColorIn[0].xyz);
	
//	if ((color.x != 0.0)  || (color.y != 0.0) || (color.z != 0.0)) 
	{ 
		gl_Position = gl_ModelViewProjectionMatrix*gl_PositionIn[0];
		EmitVertex();
		gl_Position = gl_ModelViewProjectionMatrix*gl_FrontColorIn[0];
		EmitVertex();
		gl_Position = gl_ModelViewProjectionMatrix*gl_BackColorIn[0];
		EmitVertex();	
		EndPrimitive();
	}
	/* finally color vector moves to fragment data */
}
