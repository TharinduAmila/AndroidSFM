package com.example.sfm;

import java.util.EventListener;

import android.annotation.SuppressLint;
import android.content.Context;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import android.util.Log;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;

@SuppressLint("ClickableViewAccessibility")
public class My3DView extends GLSurfaceView implements EventListener{
	private ScaleGestureDetector mScaleDetector;
	public My3DView(Context context, AttributeSet attrs) {
		super(context, attrs);
		// TODO Auto-generated constructor stub
		mScaleDetector = new ScaleGestureDetector(context, new ScaleListener());
	}

	public My3DView(Context context) {
		super(context);
		mScaleDetector = new ScaleGestureDetector(context, new ScaleListener());
	}
	@Override
    public boolean onTouchEvent(MotionEvent event)
    {
        mScaleDetector.onTouchEvent(event);
        Log.e("Scaling", ""+OpenGLRenderer.scale);
         return true;
    }
	private class ScaleListener extends ScaleGestureDetector.SimpleOnScaleGestureListener {
		@Override
		public boolean onScale(ScaleGestureDetector detector) {
		OpenGLRenderer.scale *= detector.getScaleFactor();

		// Don't let the object get too small or too large.
		OpenGLRenderer.scale = Math.min(0f, Math.max(OpenGLRenderer.scale, -1000.0f));

		invalidate();
		return true;
		}
		}
}
