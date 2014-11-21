package com.example.sfm;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import android.opengl.GLSurfaceView.Renderer;
import android.opengl.GLU;
 
public class OpenGLRenderer implements Renderer {
		private Points points;
		public static float scale=-5.0f;
		public float mCubeRotation=0.f,y=1.0f,z=0,angle=0;
		public OpenGLRenderer( ){
		   //do nothing 
		}
		
		@Override
		public void onDrawFrame( final GL10 gl ) {
			gl.glClear(GL10.GL_COLOR_BUFFER_BIT | GL10.GL_DEPTH_BUFFER_BIT);        
            gl.glLoadIdentity();
            
            gl.glTranslatef(0.0f, 0.0f, scale);
            gl.glRotatef(mCubeRotation, 0.0f, 1.0f, 0.0f);
                
            points.draw(gl);
            gl.glLoadIdentity();                                    
                
            mCubeRotation -= 0.25f; 
		}


		@Override
		public void onSurfaceChanged( final GL10 gl, final int width, final int height ) {
		    gl.glViewport( 0, 0, width, height );   
		    gl.glMatrixMode( GL10.GL_PROJECTION );  
		    gl.glLoadIdentity();       
		    //check to set far and near
		    GLU.gluPerspective( gl, 50.0f, (float)width / (float)height, 0.1f, 10000f );

		    gl.glMatrixMode( GL10.GL_MODELVIEW ); 
		    gl.glLoadIdentity();                    

		}

		@Override
		public void onSurfaceCreated( final GL10 gl, final EGLConfig config ) {
			
			gl.glClearDepthf(1.0f);    
		    points = new Points();

		}
}