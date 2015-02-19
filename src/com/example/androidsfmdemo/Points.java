package com.example.androidsfmdemo;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import javax.microedition.khronos.opengles.GL10;

import android.util.Log;

public class Points {
	double[] vertices;
	float[] fvertices;
	float[] colorArray;
	FloatBuffer vertexBuffer;
	FloatBuffer colorBuffer;
	private native double[] getSparsePointCloud();
	private native float[] getSparseColorCloud();
	public Points(){
	    updateVerticesBuffer();
	    updateColorBuffer();
	}
	public void updateVerticesBuffer(){
		vertices = getSparsePointCloud();
		fvertices = new float[vertices.length];
	    for (int i = 0; i < vertices.length; i+=3) {
	    	fvertices[i] = (float)vertices[i];
	    	fvertices[i+1] = (float)vertices[i+1];
	    	fvertices[i+2] = (float)vertices[i+2];
		}
	    ByteBuffer byteBuf = ByteBuffer.allocateDirect( vertices.length *4 );
	    byteBuf.order( ByteOrder.nativeOrder() );
	    vertexBuffer = byteBuf.asFloatBuffer();
	    vertexBuffer.put( fvertices );
	    vertexBuffer.position( 0 );
	}
	public void updateColorBuffer(){
		colorArray = getSparseColorCloud();
		//Log.e("Colors", " "+colorArray.length);
		ByteBuffer cbb = ByteBuffer.allocateDirect(colorArray.length * 4);
	    cbb.order(ByteOrder.nativeOrder());
	    colorBuffer = cbb.asFloatBuffer();
	    colorBuffer.put(colorArray);
	    colorBuffer.position(0);
	}
	
	public void draw( final GL10 gl ) {     
		gl.glColorPointer(4, GL10.GL_FLOAT, 0, colorBuffer); // NEW LINE ADDED.
		gl.glVertexPointer(3, GL10.GL_FLOAT, 0, vertexBuffer);
		
	    gl.glEnableClientState( GL10.GL_VERTEX_ARRAY );
	    gl.glEnableClientState(GL10.GL_COLOR_ARRAY); // NEW LINE ADDED.
	    /**point size*/
	    gl.glPointSize(4);
	    gl.glDrawArrays(GL10.GL_POINTS, 0, fvertices.length/3);
	    gl.glDisableClientState(GL10.GL_COLOR_ARRAY);
	    gl.glDisableClientState( GL10.GL_VERTEX_ARRAY );
	}
}
