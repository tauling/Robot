package robot;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import robot.generated.R;
import robot.navigate.Robot;
import robot.opencv.ColorBlobDetector;

import jp.ksksue.driver.serial.FTDriver;
import android.app.Activity;
import android.graphics.Camera;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.view.View.OnTouchListener;
import android.widget.ScrollView;
import android.widget.TextView;

public class MainActivity extends Activity implements OnTouchListener, CvCameraViewListener2  {

	// TODO: Create a class "Robot" with public and private methods
	
	// TODO: Working Demos -> rename to Examination Task 1
	
	// TODO: Fix: (x,y) are switched in move to goal methods
	
	// TODO: Change orientation; (0,0,0) means, that the robot is facing to the right

	// TODO: Allow the user to enter values in the app.

	// TODO: Add button to stop all threads

	// TODO: Add a function that allows to drive curves (and updates odometry)

	// TODO: Detect green and red blobs

	// TODO: Detect a ball and calculate it's lowest position (where it touches
	// the surface)

	// TODO: Move to the ball and use the robot's cage to catch it.

	// TODO: Detect multiple balls at the same time

	// TODO: Also detect blue, yellow, black and white blobs
	
	// TODO: implement MoveToTarget(x,y,theta) (ignoring obstacles)

	// TODO: explore workspace 

	// TODO: catch ball -> drive to target corner
	
	// TODO: Calculate the homography matrix using the function Mat getHomographyMatrix(Mat mRgba) (see below). It uses a chessboard pattern as shown in the picture. Add a new button to the menu for performing this action. (You will receive sheets with the correct chessboard pattern.)
	
	// TODO: Calculate distance and angle from the robot to the bottom point of a detected blob using your homography matrix. Display it in a live feed.
	
	// TODO: Detect multiple objects (of identical or distinct colors) and find their bottom points using the homography matrix.

	// TODO: Cage ball and move it to a given position
	
	// TODO: Explore workspace and remember positions of all balls
	
	
	private TextView textLog;
	
	protected Robot robot;
	
	
	/**
	 * Connects to the robot when app is started and initializes the position of
	 * the robot's bar.
	 * 
	 * @param savedInstanceState
	 */
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);

		textLog = (TextView) findViewById(R.id.textLog);
		textLog.setMovementMethod(new ScrollingMovementMethod());
		ScrollView svLog = (ScrollView) findViewById(R.id.scrollMe);

		FTDriver com = new FTDriver((UsbManager) getSystemService(USB_SERVICE));
		
		robot = new Robot(textLog, svLog, com);
		robot.connect();

        
        // TODO: The commented code right below belongs to the former ColorBlobDetectionActivity

//        Log.i(TAG, "called onCreate");
  //      requestWindowFeature(Window.FEATURE_NO_TITLE);
   //     getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);


        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
        mOpenCvCameraView.setCvCameraViewListener(this);
//		Log.i(TAG, "Instantiated new " + this.getClass());
	}


	public void buttonMoveToGoalN3_onClick(View v) {
		robot.resetPosition();
		Thread t = new Thread() {

			@Override
			public void run() {
				robot.moveToGoalNaive3(150, 150, 45);
			};
		};

		t.start();
	}



	public void buttonFindSensorIDs_onClick(View v) {
		try {
			robot.findSensorIDs();
		} catch (Exception e) {
		}
	}

	public void buttonOneMeter_onClick(View v) {
		Thread t = new Thread() {

			@Override
			public void run() {
				robot.moveRobot(100);
			};
		};

		t.start();
	}

	public void buttonOneMeterDriveByVel_onClick(View v) {
		Thread t = new Thread() {

			@Override
			public void run() {
				robot.moveByVelocity(100, false);
			};
		};

		t.start();
	}

	public void button360Deg_onClick(View v) {
		Thread t = new Thread() {

			@Override
			public void run() {
				robot.turnRobot(180, 'r');
				robot.turnRobot(180, 'r');
			};
		};

		t.start();
	}
	
	public void buttonMinus_onClick(View v) {
		robot.moveBar('-');
	}

	public void buttonPlus_onClick(View v) {
		robot.moveBar('+');
	}

	public void buttonSensor_onClick(View v) {
		Map<String, Integer> measurement = new HashMap<String, Integer>();
		measurement = robot.getDistance();
		for (Map.Entry<String, Integer> entry : measurement.entrySet()) {
			textLog.append((entry.getKey() + entry.getValue() + "\n"));
		}
	}

	public void buttonEightZero_onClick(View v) {
		robot.resetPosition();
		Thread t = new Thread() {

			@Override
			public void run() {
				robot.moveSquare(50, 'r', 0);
				robot.moveSquare(50, 'l', 0);
			};
		};
		t.start();
	}

	public void buttonEightOne_onClick(View v) {
		robot.resetPosition();
		Thread t = new Thread() {

			@Override
			public void run() {
				if (robot.moveSquare(50, 'r', 1)) {
					robot.moveSquare(50, 'l', 1);
				}
			};
		};
		t.start();
	}

	public void buttonEightTwo_onClick(View v) {
		robot.resetPosition();
		Thread t = new Thread() {

			@Override
			public void run() {
				robot.moveSquare(50, 'r', 2);
				robot.moveSquare(50, 'l', 2);
			};
		};
		t.start();
	}

	public void buttonMLineDemo_onClick(View v) {
		robot.resetPosition();
		Thread t = new Thread() {

			@Override
			public void run() {
				int x = 200;
				int y = 200;
				int theta = 45;
				robot.turnRobotBalanced(90, 'r');
				robot.moveByVelocity(100, true);
				robot.turnRobotBalanced(135, 'l');
				robot.driveToIntersectionMLine(150, x, y);
				robot.robotSetLeds(0,0);
				robot.robotSetLeds(127,127);
				robot.robotSetLeds(0,0);
				robot.robotSetLeds(127,127);
				robot.robotSetLeds(0,0);
				robot.moveToGoalNaive2(x, y, theta);
			};
		};
		t.start();
	}

	public void buttonmoveToGoalN2_onClick(View v) {
		robot.resetPosition();
		Thread t = new Thread() {

			@Override
			public void run() {
				robot.moveToGoalNaive2(250, 130, 45);
			};
		};
		t.start();
	}

	public void buttonDriveAndRead_onClick(View v) {
		robot.resetPosition();
		Thread t = new Thread() {

			@Override
			public void run() {
				robot.driveAndRead();
			};
		};

		t.start();
	}

	public void buttonTest_onClick(View v) {

		Thread t = new Thread() {

			@Override
			public void run() {
				
			};
		};

		t.start();
	}

	public void buttonTest2_onClick(View v) {

		Thread t = new Thread() {

			@Override
			public void run() {
				
			};
		};

		t.start();
	}

	public void button3_onClick(View v) {
		// TODO: Launch get Homography
	}
	
	
	// TODO: All the things below belong to the former ColorBlobDetectorActivity. Clean up and migrate.
    private static final String  TAG              = "OCVSample::Activity";

    private boolean              mIsColorSelected = false;
    private Mat                  mRgba;
    private Scalar               mBlobColorRgba;
    private Scalar               mBlobColorHsv;
    private ColorBlobDetector    mDetector;
    private Mat                  mSpectrum;
    private Size                 SPECTRUM_SIZE;
    private Scalar               CONTOUR_COLOR;

    private CameraBridgeViewBase mOpenCvCameraView;

    private BaseLoaderCallback  mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
 //                   Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                    mOpenCvCameraView.setOnTouchListener(MainActivity.this);
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    @Override
    public void onPause()
    {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public void onResume()
    {
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_3, this, mLoaderCallback);
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mDetector = new ColorBlobDetector();
        mSpectrum = new Mat();
        mBlobColorRgba = new Scalar(255);
        mBlobColorHsv = new Scalar(255);
        SPECTRUM_SIZE = new Size(200, 64);
        CONTOUR_COLOR = new Scalar(255,0,0,255);
    }

    public void onCameraViewStopped() {
        mRgba.release();
    }

    public boolean onTouch(View v, MotionEvent event) {
        int cols = mRgba.cols();
        int rows = mRgba.rows();

        int xOffset = (mOpenCvCameraView.getWidth() - cols) / 2;
        int yOffset = (mOpenCvCameraView.getHeight() - rows) / 2;

        int x = (int)event.getX() - xOffset;
        int y = (int)event.getY() - yOffset;

        Log.i(TAG, "Touch image coordinates: (" + x + ", " + y + ")");

        if ((x < 0) || (y < 0) || (x > cols) || (y > rows)) return false;

        Rect touchedRect = new Rect();

        touchedRect.x = (x>4) ? x-4 : 0;
        touchedRect.y = (y>4) ? y-4 : 0;

        touchedRect.width = (x+4 < cols) ? x + 4 - touchedRect.x : cols - touchedRect.x;
        touchedRect.height = (y+4 < rows) ? y + 4 - touchedRect.y : rows - touchedRect.y;

        Mat touchedRegionRgba = mRgba.submat(touchedRect);

        Mat touchedRegionHsv = new Mat();
        Imgproc.cvtColor(touchedRegionRgba, touchedRegionHsv, Imgproc.COLOR_RGB2HSV_FULL);

        // Calculate average color of touched region
        mBlobColorHsv = Core.sumElems(touchedRegionHsv);
        int pointCount = touchedRect.width*touchedRect.height;
        for (int i = 0; i < mBlobColorHsv.val.length; i++)
            mBlobColorHsv.val[i] /= pointCount;

        mBlobColorRgba = converScalarHsv2Rgba(mBlobColorHsv);

        Log.i(TAG, "Touched rgba color: (" + mBlobColorRgba.val[0] + ", " + mBlobColorRgba.val[1] +
                ", " + mBlobColorRgba.val[2] + ", " + mBlobColorRgba.val[3] + ")");

        mDetector.setHsvColor(mBlobColorHsv);

        Imgproc.resize(mDetector.getSpectrum(), mSpectrum, SPECTRUM_SIZE);

        mIsColorSelected = true;

        touchedRegionRgba.release();
        touchedRegionHsv.release();

        return false; // don't need subsequent touch events
    }
    
    //TODO: add comment
    public Point computeCenterPt(List<MatOfPoint> contours){
    	if(contours.isEmpty()){
    		return (new Point(-200.0,-200.0));
    	}
    	int avgX = 0, avgY = 0,count = 0;
    	for(int i=0;i<contours.size();i++){
    		List<Point> pts = contours.get(i).toList();
    		for(Point p : pts){
    			avgX += p.x;
    			avgY += p.y;
    			count++;
    		}
    	}
    	Point ptCenter = new Point(avgX/count,avgY/count);
    	return ptCenter;
    }
    
    public int distPointToPoint(Point p1, Point p2){
    	return (int) Math.sqrt(Math.pow(p2.x - p1.x, 2)+Math.pow(p2.y-p1.y,2));
    }
    
    //TODO: add comment
    /**
     * should maybe return a boolean...
     */
    public void catchObstacle(int radius){
    	int handySurface = 1920*1080;
    	double ballSurface = Math.pow(radius, 2)*Math.PI;
    	if(ballSurface/handySurface >= 70){
    		//lower bar
    	}
    }
    
    //TODO: add comment
    public int computeRadius(List<MatOfPoint> contours, Point center){
    	if(contours.isEmpty()){
    		return 0;
    	}
    	int distToCenter = 0;
    	for(int i=0;i<contours.size();i++){
    		List<Point> pts = contours.get(i).toList();
    		for(Point p : pts ){
    			if(distPointToPoint(p, center) > distToCenter){
    				distToCenter = distPointToPoint(p, center);
    			}
    		}
    	}
    	return distToCenter;
    }

    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        mRgba = inputFrame.rgba();
        //Mat mRgbaT = inputFrame.rgba();

        if (mIsColorSelected) {
            mDetector.process(mRgba);
            List<MatOfPoint> contours = mDetector.getContours();
            Point center = computeCenterPt(contours);
            int rad = computeRadius(contours,center);
            Log.e(TAG, "centerPoint:"+center.toString());
            Log.e(TAG, "Contours count: " + contours.size());
            Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR);
            int radius = Math.min(mRgba.cols() / 4, mRgba.rows() / 4);
            Scalar color = new Scalar(128);
            Core.circle(mRgba, center, 10, new Scalar(20),-1);
            Core.circle(mRgba, center, rad, color,5);
            Mat colorLabel = mRgba.submat(4, 68, 4, 68);
            colorLabel.setTo(mBlobColorRgba);

            Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
            mSpectrum.copyTo(spectrumLabel);
//            mRgbaT = mRgba.t();
            
//            Core.flip(mRgba.t(),mRgbaT,1);
//            Imgproc.resize(mRgbaT, mRgbaT, mRgba.size());
        }

        return mRgba;
    }

    private Scalar converScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

        return new Scalar(pointMatRgba.get(0, 0));
    }
    
    protected void setDisplayOrientation(Camera camera, int angle){
        Method downPolymorphic;
        try
        {
            downPolymorphic = camera.getClass().getMethod("setDisplayOrientation", new Class[] { int.class });
            if (downPolymorphic != null)
                downPolymorphic.invoke(camera, new Object[] { angle });
        }
        catch (Exception e1)
        {
        }
    }
    
}
