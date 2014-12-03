package com.example.sfm;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import javax.microedition.khronos.opengles.GL10;

import android.util.Log;

public class Points {
	private double[] vertices;
	private float[] fvertices;
	float[] colorArray;
	private FloatBuffer vertexBuffer;
	private FloatBuffer colorBuffer;
	public native double[] getPointsArray();
	public native float[] getColorsArray();
	public Points(){

	    //this.vertices=pointsArray;
		this.vertices = getPointsArray();
		fvertices = new float[vertices.length];
	    colorArray = getColorsArray();
	    Log.e("Size", ""+colorArray.length);
	    String show ="",colo="";
	    for (int i = 0,j=0; i < vertices.length; i+=3,j+=4) {
	    	fvertices[i] = (float)vertices[i];
	    	fvertices[i+1] = (float)vertices[i+1];
	    	fvertices[i+2] = (float)vertices[i+2];
	    	show += "["+fvertices[i]+","+fvertices[i+1]+","+fvertices[i+2]+"] ";
	    	colo += "["+colorArray[j]+","+colorArray[j+1]+","+colorArray[j+2]+","+colorArray[j+3]+"] ";
		}
	    Log.e("Point List", show);
	    Log.e("Color List", colo);
	    ByteBuffer byteBuf = ByteBuffer.allocateDirect( vertices.length *4 );
	    byteBuf.order( ByteOrder.nativeOrder() );
	    vertexBuffer = byteBuf.asFloatBuffer();
	    vertexBuffer.put( fvertices );
	    vertexBuffer.position( 0 );
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
	    //gl.glEnable( GL10.GL_POINT_SMOOTH );
	    //gl.glEnable( GL10.GL_BLEND );
	    //gl.glBlendFunc( GL10.GL_SRC_ALPHA, GL10.GL_ONE_MINUS_SRC_ALPHA );
	 // Enable the color array buffer to be used during rendering.
	    // Point out the where the color buffer is.
	    /**point size*/
	    gl.glPointSize(4);
	    gl.glDrawArrays(GL10.GL_POINTS, 0, vertices.length/3);
	    gl.glDisableClientState( GL10.GL_VERTEX_ARRAY );
	    gl.glDisableClientState(GL10.GL_COLOR_ARRAY);

	}
}
