package com.example.androidsfmdemo;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;

import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.ActionBarActivity;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.SurfaceView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;

public class MainActivity extends ActionBarActivity implements
		CvCameraViewListener2 {
	// general data holders
	private Mat imageHolder;
	private int gcCounter = 0;
	private final int imageLimit = 9;
	private CustomJavaCameraView mOpenCvCameraView;

	// Native methods
	private native void sendImageToNative(long addrOut);

	private native void clearAllData();

	private native void viewDataFromNative(long addrOut);

	private native int getImageCount();

	private native int runSparseReconstruction();

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		mOpenCvCameraView = (CustomJavaCameraView) findViewById(R.id.javaCameraView1);
		mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
		mOpenCvCameraView.setMaxFrameSize(700, 600);
		mOpenCvCameraView.setCvCameraViewListener(this);
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.main, menu);
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

	@Override
	public void onCameraViewStarted(int width, int height) {
		// TODO Auto-generated method stub

	}

	@Override
	public void onCameraViewStopped() {
		// TODO Auto-generated method stub

	}

	@Override
	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
		// TODO Auto-generated method stub
		// if(gcCounter>gcCallLimit){ //commented out to check if the memory
		// issue is handled correctly now
		// System.gc();
		// gcCounter=0;
		// }else{
		// gcCounter++;
		// }
		if (imageHolder == null) {
			imageHolder = inputFrame.rgba().clone();
		} else {
			inputFrame.rgba().copyTo(imageHolder);
		}// imageHolder = inputFrame.rgba().clone(); // creates memory leak
		Mat out = inputFrame.rgba();
		viewDataFromNative(out.getNativeObjAddr());
		return out;
	}

	@Override
	public void onResume() {
		super.onResume();
		// loading opencv natives and android libraries
		OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_9, this,
				mLoaderCallback);
	}

	@Override
	public void onPause() {
		super.onPause();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	@Override
	public void onDestroy() {
		super.onDestroy();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
		@Override
		public void onManagerConnected(int status) {
			switch (status) {
			case LoaderCallbackInterface.SUCCESS: {
				Log.i("OpenCV", "OpenCV loaded successfully");
				System.loadLibrary("AndroidSFM"); // load native library
				mOpenCvCameraView.enableView();
			}
				break;
			default: {
				super.onManagerConnected(status);
			}
				break;
			}
		}
	};

	// Ui input handlers
	public void sendImage(View view) {
			sendImageToNative(imageHolder.getNativeObjAddr());
		if (getImageCount() > 1) {
			Button sparse = (Button) findViewById(R.id.button1);
			sparse.setEnabled(true);
		}
		if(getImageCount() == imageLimit){
			Button capture = (Button) findViewById(R.id.button2);
			capture.setEnabled(false);
		}
	}

	public void resetSystem(View view) {
		clearAllData();
		Button sparse = (Button) findViewById(R.id.button1);
		Button capture = (Button) findViewById(R.id.button2);
		sparse.setEnabled(false);
		capture.setEnabled(true);
	}

	public void doSparseReconstruction(View view) {
		Button capture = (Button) findViewById(R.id.button2);
		int success = runSparseReconstruction();
		if (success == 1) {
			capture.setEnabled(false);
			Intent intent = new Intent(this, Viewer3D.class);
			startActivity(intent);
		}else{
			resetSystem(view);
		}
	}
}
