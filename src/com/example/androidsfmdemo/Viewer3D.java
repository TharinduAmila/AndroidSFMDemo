package com.example.androidsfmdemo;

import android.app.Activity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.widget.Button;

public class Viewer3D extends Activity {
	private My3DView m3dView;
	private OpenGLRenderer renderer;
	private Button up,down,left,right;
	private final int u=0,d=1,l=2,r=3;
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		renderer = new OpenGLRenderer();
		setContentView(R.layout.activity_viewer3_d);
		m3dView = (My3DView) findViewById(R.id.surfaceView1);
		m3dView.setRenderer(renderer);
		up = (Button) findViewById(R.id.up);
		down = (Button) findViewById(R.id.down);
		left = (Button) findViewById(R.id.left);
		right = (Button) findViewById(R.id.right);
		up.setOnTouchListener(new ButtonTouchListener(u,up));
		down.setOnTouchListener(new ButtonTouchListener(d,down));
		left.setOnTouchListener(new ButtonTouchListener(l,left));
		right.setOnTouchListener(new ButtonTouchListener(r,right));
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.viewer3_d, menu);
		return true;
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		// Handle action bar item clicks here. The action bar will
		// automatically handle clicks on the Home/Up button, so long
		// as you specify a parent activity in AndroidManifest.xml.
		int id = item.getItemId();
		if (id == R.id.action_settings) {
			return true;
		}
		return super.onOptionsItemSelected(item);
	}
	
	private class AngleChanger extends Thread{
		private int type = -1;
		float speed;
		volatile boolean run; 
		public AngleChanger(int i){
			run = false;
			type = i;
			speed = 1F;
		}
		@Override
		public void run() {
			// TODO Auto-generated method stub
			run = true;
			while(run){
			switch (type){
			case u:
				renderer.camAngleY+=speed;
				if(renderer.camAngleY>180){
					renderer.camAngleY=180.0f;
				}
				break;
			case d:
				renderer.camAngleY-=speed;
				if(renderer.camAngleY<0){
					renderer.camAngleY=0.1f;
				}
				break;
			case l:
				renderer.camAngleX+=speed;
				if(renderer.camAngleX>360.0F){
					renderer.camAngleX=0.0f;
				}
				break;
			case r:
				renderer.camAngleX-=speed;
				if(renderer.camAngleX<0.F){
					renderer.camAngleX = 360.0f;
				}
				break;
			}
			try {
				sleep(10);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			}
		}
		
	}
	private class ButtonTouchListener implements OnTouchListener {
		private AngleChanger valueChanger;
		private int type;
		public ButtonTouchListener(int in,Button b){
			valueChanger = null;
			type = in;
		}
		@Override
		public boolean onTouch(View arg0, MotionEvent event) {
			// TODO Auto-generated method stub
			if(event.getAction() == MotionEvent.ACTION_DOWN) {
				if(valueChanger!=null && valueChanger.run==true){
					valueChanger.run = false;
				}
				
				valueChanger = new AngleChanger(type);
				valueChanger.start();
				
	        }else if (event.getAction() == MotionEvent.ACTION_UP) {
	           if(valueChanger!=null &&  valueChanger.run==true){
	        	valueChanger.run = false;
	           }
	        }
			return true;
		}

	}
}
