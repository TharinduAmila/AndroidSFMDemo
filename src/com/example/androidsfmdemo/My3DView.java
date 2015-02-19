package com.example.androidsfmdemo;

import java.util.EventListener;

import android.annotation.SuppressLint;
import android.content.Context;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import android.util.Log;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;

@SuppressLint("ClickableViewAccessibility")
public class My3DView extends GLSurfaceView implements EventListener {
	private ScaleGestureDetector mScaleDetector;
	private OpenGLRenderer myRendered;

	public My3DView(Context context, AttributeSet attrs) {
		super(context, attrs);
		// TODO Auto-generated constructor stub
		mScaleDetector = new ScaleGestureDetector(context, new ScaleListener());
	}

	@Override
	public void setRenderer(Renderer renderer) {
		// TODO Auto-generated method stub
		myRendered = (OpenGLRenderer) renderer;
		super.setRenderer(renderer);
	}

	public My3DView(Context context) {
		super(context);
		mScaleDetector = new ScaleGestureDetector(context, new ScaleListener());
	}

	float startX, startY;

	@Override
	public boolean onTouchEvent(MotionEvent event) {
		mScaleDetector.onTouchEvent(event);
		invalidate();
		return true;
		
	}

	private class ScaleListener extends
			ScaleGestureDetector.SimpleOnScaleGestureListener {
		@Override
		public boolean onScale(ScaleGestureDetector detector) {
			if(detector.getScaleFactor()>1)
				myRendered.scale+=1;
			else
				myRendered.scale-=1;
			// Don't let the object get too small or too large.
			myRendered.scale = Math.min(0f,
					Math.max(myRendered.scale, -1000.0f));
			invalidate();
			return true;
		}
	}
}
