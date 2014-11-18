package com.example.sfm;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import android.support.v7.app.ActionBarActivity;
import android.content.Intent;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.SurfaceView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;

public class MainActivity extends ActionBarActivity implements
CvCameraViewListener2{
	private CameraBridgeViewBase mOpenCvCameraView;
	Mat image1,image2,matchedMat;
	private int matcher = 0;
	boolean matched=false;
	double reErr=0;
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		System.loadLibrary("sfmlib");
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		Log.i("OpenCvStuff", "Trying to load OpenCV library");
	    if (!OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_9, this,
				mLoaderCallback))
	    {
	      Log.e("OpenCvStuff", "Cannot connect to OpenCV Manager");
	    }
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.CameraView);
		mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
		mOpenCvCameraView.setCvCameraViewListener(this);
		mOpenCvCameraView.setMaxFrameSize(1280, 800);
		//image1 = new Mat();
		//image2 = new Mat();
	}
	public native double updateCurrentImage(long addrIn);
	public native void setImage1(long addrOut);
	public native void setImage2(long addrOut);
	public native int match(long addrOut);
	public native void setMatcher(int in);
	public native void ClearAll();
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
		reErr = updateCurrentImage(inputFrame.rgba().getNativeObjAddr());
		if(!matched)
		return inputFrame.rgba();
		else{
			Log.d("Re Error",""+reErr);
			return matchedMat;
		}
	}
	@Override
	public void onResume() {
		super.onResume();
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
	/** Called when the user touches the button */
	int count =0;
	public void setImage(View view) {
	    if(count==0){
	    	image1 = new Mat();
	    	setImage1(image1.getNativeObjAddr());
	    	Bitmap bm = Bitmap.createBitmap(image1.cols(), image1.rows(),Bitmap.Config.ARGB_8888);
	        Utils.matToBitmap(image1, bm);

	        // find the imageview and draw it!
	        ImageView iv = (ImageView) findViewById(R.id.imageView1);
	        iv.setImageBitmap(bm);
	        count++;
	    }else{
	    	image2 = new Mat();
	    	setImage2(image2.getNativeObjAddr());
	    	Bitmap bm = Bitmap.createBitmap(image2.cols(), image2.rows(),Bitmap.Config.ARGB_8888);
	        Utils.matToBitmap(image2, bm);

	        // find the imageview and draw it!
	        ImageView iv = (ImageView) findViewById(R.id.ImageView2);
	        iv.setImageBitmap(bm);
	        Button bt = (Button)view;
	        bt.setEnabled(false);
	    }
	}
	public void clearImage(View view){
		Button bt = (Button)findViewById(R.id.button1);
		bt.setEnabled(true);
		Bitmap bm = Bitmap.createBitmap(200, 100,Bitmap.Config.ARGB_8888);
		ImageView iv = (ImageView) findViewById(R.id.imageView1);
        iv.setImageBitmap(bm);
        iv = (ImageView)findViewById(R.id.ImageView2);
        iv.setImageBitmap(bm);
        count =0;
        matched = false;
        ClearAll();
	}
	public void runMatcher(View view){
		matchedMat = new Mat();
		if(match(matchedMat.getNativeObjAddr())==0){
			clearImage(view);
			return;
		}
		Imgproc.resize(matchedMat,matchedMat,image1.size());
        matched = true;
	}
	public void goTo3DView(View view) {
		Intent intent = new Intent(this, Viewer3dActivity.class);
		startActivity(intent);
	}
	public void changeMatcherType(View view){
		matcher++;
		matcher = matcher%3;
		TextView text =  (TextView)findViewById(R.id.textView1);
		switch(matcher){
		case 0:
			setMatcher(0);
			text.setText("ORB feature Matcher");
			break;
		case 1:
			setMatcher(1);
			text.setText("ORB feature+KLT hybrid Matcher");
			break;
		case 2:
			setMatcher(2);
			text.setText("Dense OF Matcher");
			break;
		}
	}
}
