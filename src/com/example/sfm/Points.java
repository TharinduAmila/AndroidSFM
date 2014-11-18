package com.example.sfm;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.DoubleBuffer;
import java.nio.FloatBuffer;

import javax.microedition.khronos.opengles.GL10;

import android.util.Log;

public class Points {
	private double[] vertices;
	private float[] fvertices;
	private FloatBuffer vertexBuffer;
	private FloatBuffer colorBuffer;
	public native double[] getPointsArray();
	public Points( final float[] pointsArray ){

	    //this.vertices=pointsArray;
		this.vertices = getPointsArray();
		fvertices = new float[vertices.length];
	    float[] colorArray = new float[(vertices.length/3)*4];
	    String show ="";
	    for (int i = 0,j=0; i < colorArray.length; i+=4,j+=3) {
	    	colorArray[i] = (float)Math.random(); //0f, 0f, 1f
	    	colorArray[i+1] = (float)Math.random();
	    	colorArray[i+2] = (float)Math.random();
	    	colorArray[i+3] = 1f;
	    	fvertices[j] = (float)vertices[j];
	    	fvertices[j+1] = (float)vertices[j+1];
	    	fvertices[j+2] = (float)vertices[j+2];
	    	show += "["+fvertices[j]+","+fvertices[j+1]+","+fvertices[j+2]+"] ";
		}
	    Log.e("Point List", show);
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
