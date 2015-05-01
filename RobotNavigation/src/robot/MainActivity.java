package robot;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
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
import robot.navigate.Ball;
import robot.navigate.Position;
import robot.navigate.Robot;
import robot.opencv.ColorBlobDetector;

import jp.ksksue.driver.serial.FTDriver;
import android.app.Activity;
import android.graphics.Camera;
import android.hardware.usb.UsbManager;
import android.media.Image;
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

public class MainActivity extends Activity implements OnTouchListener,
		CvCameraViewListener2 {

	// TODO: Create a class "Robot" with public and private methods

	// TODO: Working Demos -> rename to Examination Task 1

	// TODO: Fix: (x,y) are switched in move to goal methods

	// TODO: Change orientation; (0,0,0) means, that the robot is facing to the
	// right

	// TODO: Allow the user to enter values in the app.

	// TODO: Add button to stop all threads

	// TODO: Add a function that allows to drive curves (and updates odometry)

	// TODO: Detect green and red blobs

	// TODO: Detect a ball and calculate it's lowest position (where it touches
	// the surface)

	// TODO: Move to the ball and use the robot's cage to catch it.

	// TODO: Detect multiple balls at the same time

	// TODO: Also detect blue, yellow, black and white blobs

	// TODO: explore workspace

	// TODO: catch ball -> drive to target corner

	// TODO: Calculate the homography matrix using the function Mat
	// getHomographyMatrix(Mat mRgba) (see below). It uses a chessboard pattern
	// as shown in the picture. Add a new button to the menu for performing this
	// action. (You will receive sheets with the correct chessboard pattern.)

	// TODO: Calculate distance and angle from the robot to the bottom point of
	// a detected blob using your homography matrix. Display it in a live feed.

	// TODO: Detect multiple objects (of identical or distinct colors) and find
	// their bottom points using the homography matrix.

	// TODO: Cage ball and move it to a given position

	// TODO: Explore workspace and remember positions of all balls

	private TextView textLog;

	private Robot robot;

	private Mat homographyMatrix;

	private List<Scalar> hsvColors = new LinkedList<Scalar>();

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

		// TODO: The commented code right below belongs to the former
		// ColorBlobDetectionActivity

		// Log.i(TAG, "called onCreate");
		// requestWindowFeature(Window.FEATURE_NO_TITLE);
		// getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

		mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
		mOpenCvCameraView.setCvCameraViewListener(this);
		// Log.i(TAG, "Instantiated new " + this.getClass());
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

	public void buttonGetBlob_onClick(View v) {
		Mat src = new Mat(1, 1, CvType.CV_32FC2);
		Mat dest = new Mat(1, 1, CvType.CV_32FC2);
		Point lowestPoint = lowestPt();
		robot.writeLog("Found ball on cam at x = " + lowestPoint.x
				+ " and y = " + lowestPoint.y);
		src.put(0, 0, new double[] { lowestPoint.x, lowestPoint.y }); // ps is a
																		// point
																		// in
																		// image
																		// coordinates
		Core.perspectiveTransform(src, dest, homographyMatrix);
		Point dest_point = new Point(dest.get(0, 0)[0], dest.get(0, 0)[1]);
		robot.writeLog("Found Blob at x = " + dest_point.x + " and y = "
				+ dest_point.y);
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

	public void button360DegByVel_onClick(View v) {
		Thread t = new Thread() {

			@Override
			public void run() {
				robot.turnByVelocity(180, 'r');
				robot.turnByVelocity(180, 'r');
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
				robot.robotSetLeds(0, 0);
				robot.robotSetLeds(127, 127);
				robot.robotSetLeds(0, 0);
				robot.robotSetLeds(127, 127);
				robot.robotSetLeds(0, 0);
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
				robot.robotSetBar(120);
				findAndDeliverBall();
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
		Thread t = new Thread() {

			@Override
			public void run() {
				homographyMatrix = new Mat();
				int i = 0;
				while (homographyMatrix.empty()) {
					try {
						robot.writeLog(i++
								+ ". try: searching homography Matrix.");
						homographyMatrix = mDetector
								.getHomographyMatrix(mRgbaOutput);
						Thread.sleep(1000);
					} catch (InterruptedException e) {
						// do nothing
					}
				}
				robot.writeLog("Homography Matrix found.");
			};
		};

		t.start();
	}

	public void ButtonfindCirclesOnClick(View v) {

		Thread t = new Thread() {

			@Override
			public void run() {
				List<Point> circles = findCirclesOnCamera();
				if (circles.size() > 0)
					getGroundPlaneCoordinates(circles.get(0));
			};
		};

		t.start();
	}
	
	public void ButtonEmptyBrain(View v) {

		Thread t = new Thread() {

			@Override
			public void run() {
				circleCenters = new ArrayList<Point>();
				robot.resetPosition();
				homographyMatrix = new Mat();
			};
		};

		t.start();
	}

	// TODO: All the things below belong to the former
	// ColorBlobDetectorActivity. Clean up and migrate.
	private static final String TAG = "RobotLog";

	private boolean mIsColorSelected = false;
	private Mat mRgbaOutput;
	private Mat mRgbaWork;
	private Scalar mBlobColorRgba;
	private Scalar mBlobColorHsv;
	private ColorBlobDetector mDetector;
	private Mat mSpectrum;
	private Size SPECTRUM_SIZE;
	private Scalar CONTOUR_COLOR;

	private int executionInterval = 15;

	private List<Scalar> myColors = new ArrayList<Scalar>();

	/**
	 * list which stores all found balls
	 */
	private List<Ball> foundBalls = new ArrayList<Ball>();
	
	List<Point> circleCenters = new ArrayList<Point>();

	private CameraBridgeViewBase mOpenCvCameraView;

	private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
		@Override
		public void onManagerConnected(int status) {
			switch (status) {
			case LoaderCallbackInterface.SUCCESS: {
				// Log.i(TAG, "OpenCV loaded successfully");
				mOpenCvCameraView.enableView();
				mOpenCvCameraView.setOnTouchListener(MainActivity.this);
			}
				break;
			default: {
				super.onManagerConnected(status);
			}
				break;
			}
		}
	};

	@Override
	public void onPause() {
		super.onPause();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	@Override
	public void onResume() {
		super.onResume();
		OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_3, this,
				mLoaderCallback);
	}

	public void onDestroy() {
		super.onDestroy();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	public void onCameraViewStarted(int width, int height) {
		mRgbaWork = new Mat(height, width, CvType.CV_8UC4);
		mRgbaOutput = new Mat(height, width, CvType.CV_8UC4);
		mDetector = new ColorBlobDetector();
		mSpectrum = new Mat();
		mBlobColorRgba = new Scalar(255);
		mBlobColorHsv = new Scalar(255);
		SPECTRUM_SIZE = new Size(200, 64);
		CONTOUR_COLOR = new Scalar(255, 0, 0, 255);
	}

	public void onCameraViewStopped() {
		mRgbaOutput.release();
	}

	public boolean onTouch(View v, MotionEvent event) {
		int cols = mRgbaOutput.cols();
		int rows = mRgbaOutput.rows();
		int xOffset = (mOpenCvCameraView.getWidth() - cols) / 2;
		int yOffset = (mOpenCvCameraView.getHeight() - rows) / 2;
		int x = (int) event.getX() - xOffset;
		int y = (int) event.getY() - yOffset;
		Log.i(TAG, "Touch image coordinates: (" + x + ", " + y + ")");
		if ((x < 0) || (y < 0) || (x > cols) || (y > rows))
			return false;
		Rect touchedRect = new Rect();
		touchedRect.x = (x > 4) ? x - 4 : 0;
		touchedRect.y = (y > 4) ? y - 4 : 0;
		touchedRect.width = (x + 4 < cols) ? x + 4 - touchedRect.x : cols
				- touchedRect.x;
		touchedRect.height = (y + 4 < rows) ? y + 4 - touchedRect.y : rows
				- touchedRect.y;
		Mat touchedRegionRgba = mRgbaOutput.submat(touchedRect);
		Mat touchedRegionHsv = new Mat();
		Imgproc.cvtColor(touchedRegionRgba, touchedRegionHsv,
				Imgproc.COLOR_RGB2HSV_FULL);
		// Calculate average color of touched region
		mBlobColorHsv = Core.sumElems(touchedRegionHsv);
		int pointCount = touchedRect.width * touchedRect.height;
		for (int i = 0; i < mBlobColorHsv.val.length; i++)
			mBlobColorHsv.val[i] /= pointCount;
		mBlobColorRgba = converScalarHsv2Rgba(mBlobColorHsv);
		// add mBlobColorHsv in myColors-list; ignores Rgba color
		myColors.add(mBlobColorHsv);
		Log.i(TAG, "Touched rgba color: (" + mBlobColorRgba.val[0] + ", "
				+ mBlobColorRgba.val[1] + ", " + mBlobColorRgba.val[2] + ", "
				+ mBlobColorRgba.val[3] + ")");
		Log.i(TAG, "saved colors: " + myColors.size());
		mIsColorSelected = true;
		touchedRegionRgba.release();
		touchedRegionHsv.release();
		return false; // don't need subsequent touch events
	}

	/**
	 * computes center point of given contour
	 * 
	 * @param contours
	 * @return Point (representing center point)
	 */
	public Point computeCenterPt(List<MatOfPoint> contours) {
		if (contours.isEmpty()) {
			return (new Point(-200.0, -200.0));
		}
		double avgX = 0, avgY = 0;
		int count = 0;
		for (int i = 0; i < contours.size(); i++) {
			List<Point> pts = contours.get(i).toList();
			for (Point p : pts) {
				avgX += p.x;
				avgY += p.y;
				count++;
			}
		}
		Point ptCenter = new Point(avgX / count, avgY / count);
		return ptCenter;
	}

	public double distPointToPoint(Point p1, Point p2) {
		return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
	}

	// public void circleDetection(Mat frame) {
	// // the lower this figure the more spurious circles you get
	// // 50 looks good in CANNY, but 100 is better when converting that into
	// Hough circles
	// int iCannyUpperThreshold = 100;
	// Mat grayImg = null;
	// Point pt;
	// int radius;
	//
	// grayImg = CreateImage(frame.size(), 8, 1);
	// Imgproc.HoughCircles(frame, grayImg, Imgproc.CV_HOUGH_GRADIENT, 2.0,
	// frame.rows() / 8,
	// iCannyUpperThreshold, 200, 20, 400);
	//
	// if (grayImg.cols() > 0)
	// for (int x = 0; x < Math.min(grayImg.cols(), 10); x++)
	// {
	// double vCircle[] = grayImg.get(0,x);
	//
	// if (vCircle == null)
	// break;
	//
	// pt = new Point(Math.round(vCircle[0]), Math.round(vCircle[1]));
	// radius = (int)Math.round(vCircle[2]);
	// // draw the found circle
	// // Core.circle(mRgba, pt, radius, colorRed, iLineThickness);
	//
	// // draw a cross on the centre of the circle
	// // Core.circle(mRgba, pt, 5, , );
	// }
	// }

	/**
	 * unstable method
	 * 
	 * computes Radius based on distance from most far away point to center
	 * 
	 * @param contours
	 * @param center
	 *            point
	 * @return radius
	 */
	public double computeRadius(List<MatOfPoint> contours, Point center) {
		if (contours.isEmpty()) {
			return 0;
		}
		double distToCenter = 0;
		for (int i = 0; i < contours.size(); i++) {
			List<Point> pts = contours.get(i).toList();
			for (Point p : pts) {
				if (distPointToPoint(p, center) > distToCenter) {
					distToCenter = distPointToPoint(p, center);
				}
			}
		}
		return distToCenter;
	}

	/**
	 * computes contour radius with circle surface equation
	 * 
	 * @param contours
	 * @return radius
	 */
	public int computeRadius2(List<MatOfPoint> contours) {
		if (contours.isEmpty()) {
			return 0;
		}
		int radius2 = (int) (computeContourArea(contours) / (2 * Math.PI));

		return radius2;
	}

	/**
	 * measures distance from every point to center and computes average radius
	 * 
	 * @param contours
	 * @param center
	 *            point
	 * @return radius
	 */
	public double computeRadius3(List<MatOfPoint> contours, Point center) {
		if (contours.isEmpty()) {
			return 0;
		}
		center = computeCenterPt(contours);
		double distToCenter = 0;
		int nrElements = 0;
		for (int i = 0; i < contours.size(); i++) {
			List<Point> pts = contours.get(i).toList();
			for (Point p : pts) {
				distToCenter += distPointToPoint(p, center);
			}
			nrElements += pts.size();
		}
		distToCenter = distToCenter / nrElements;

		return distToCenter;
	}

	/**
	 * computes surface of given contour (by counting points inside)
	 * 
	 * @param contours
	 * @return surface of contour (point amount)
	 */
	public int computeContourArea(List<MatOfPoint> contours) {
		if (contours.isEmpty()) {
			return 0;
		}
		int area = 0;
		for (int i = 0; i < contours.size(); i++) {
			List<Point> pts = contours.get(i).toList();
			for (Point p : pts) {
				area++;
			}
		}
		return area;
	}

	public void separateContours() {
		if (mIsColorSelected) {
			mDetector.process(mRgbaOutput);
			mDetector.getContours();
		}
	}

	// TODO: add comment; use this method in onCameraFrame
	public Point lowestPt() {
		Point lowPt = null;
		if (mIsColorSelected) {
			mDetector.process(mRgbaOutput);
			List<MatOfPoint> contours = mDetector.getContours();
			Point center = computeCenterPt(contours);
			double rad = computeRadius3(contours, center);
			robot.writeLog("Radius: " + rad);
			robot.writeLog("Center x = " + center.x + " and y = " + center.y);

			// Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR);
			// int radius = Math.min(mRgba.cols() / 4, mRgba.rows() / 4);

			lowPt = new Point(center.x, center.y + rad);
		}
		return lowPt;
	}

	/**
	 * robot aligns his body to a surrendered ball
	 * 
	 * @param ball
	 * @return TRUE, after he turned enough
	 */
	public Boolean alignToBall(Ball ball) {
		Boolean aligned = false;
		double centerXAxis = mRgbaOutput.width() / 2;
		double TOL = 2.0;
		while (!aligned) {
			Point ballCenter = ball.getBallCenterCameraFrame();
			double ballXAxis = ballCenter.x;
			double diff = centerXAxis - ballXAxis;
			if (Math.abs(diff) > TOL && diff < 0) {
				robot.turnRobot(5, 'r');
			} else if (Math.abs(diff) > TOL && diff < 0) {
				robot.turnRobot(2, 'l');
			}

		}
		return aligned;
	}

	private int frameInterval = 0;
	
	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
		mRgbaOutput = inputFrame.rgba();
		mRgbaWork = inputFrame.rgba();
		if(frameInterval >= executionInterval){
			findCirclesOnCamera();
			frameInterval = 0;
		}
		if(!circleCenters.isEmpty()){
			for(Point circleCenter:circleCenters)
				Core.circle(mRgbaOutput, circleCenter, 10, new Scalar(20), -1);
		}
		frameInterval++;

		return mRgbaOutput;
	}

	// TODO Needed?
	// TODO If so, add comment
	// TODO move to colorblog-class
	public Point computeAvgPt(ArrayList<Point> points) {
		Point avgPoint = new Point(0.0, 0.0);
		for (Point p : points) {
			avgPoint.x += p.x;
			avgPoint.y += p.y;
		}
		avgPoint.x = avgPoint.x / points.size();
		avgPoint.y = avgPoint.y / points.size();
		return avgPoint;
	}

	// TODO Needed?
	// TODO If so, add comment
	// TODO move to colorblog-class
	public Point computeTargetPt() {
		Point target = new Point();
		Mat src = new Mat(1, 1, CvType.CV_32FC2);
		Mat dest = new Mat(1, 1, CvType.CV_32FC2);
		ArrayList<Point> avgLowPt = new ArrayList<Point>();
		// do 20 measurements
		for (int i = 0; i < 20; i++) {
			avgLowPt.add(lowestPt());
		}
		Point avgPt = computeAvgPt(avgLowPt);
		double TOL = 5.0;
		for (Point pt : avgLowPt) {
			if (Math.abs(pt.x - avgPt.x) > TOL
					|| Math.abs(pt.y - avgPt.y) > TOL) {
				pt.x = avgPt.x;
				pt.y = avgPt.y;
			}
		}
		avgPt = computeAvgPt(avgLowPt);
		robot.writeLog("Found ball on cam at x = " + avgPt.x + " and y = "
				+ avgPt.y);
		src.put(0, 0, new double[] { avgPt.x, avgPt.y }); // ps is a
															// point
															// in
															// image
															// coordinates
		Core.perspectiveTransform(src, dest, homographyMatrix);
		Point dest_point = new Point(dest.get(0, 0)[0], dest.get(0, 0)[1]);
		robot.writeLog("Found Blob at x = " + dest_point.x + " and y = "
				+ dest_point.y);
		target.x = dest_point.x;
		target.y = dest_point.y;
		return target;
	}

	// TODO Needed?
	// TODO If so, add comment
	// TODO move to colorblog-class
	public void printBallInfo() {
		// Log.i(TAG, "ball amount" + myBalls.size());
		for (Ball b : foundBalls) {
			Log.i(TAG, b.getId() + " .ball position: " + b.getPosGroundPlane()
					+ " ball center: " + b.getBallCenterCameraFrame()
					+ " ball radius: " + b.getRadius());
		}
	}

	public Mat drawBalls(Mat mRgbaWithBalls) {

		int counter = 0;
		for (Ball ball : foundBalls) {

			// Just for highlighting
			Scalar color = null;
			Scalar red = new Scalar(20);
			Scalar black = new Scalar(128);
			if (counter % 2 == 0) {
				color = red;
			} else {
				color = black;
			}

			Point center = ball.getBallCenterCameraFrame();
			int rad = (int) ball.getRadius();
			Core.circle(mRgbaWithBalls, center, 10, new Scalar(20), -1);
			Core.circle(mRgbaWithBalls, center, (int) rad, color, 5);
			Point lowestPoint = new Point(center.x, center.y + rad);
			Core.circle(mRgbaWithBalls, lowestPoint, 10, color, 5);
			// Imgproc.drawContours(img, ballArea, -1, color, -1);

			counter++;
		}

		return mRgbaWithBalls;
	}

	// TODO: refactor name
	// TODO: needed?
	// TODO: If so, move to colorblog-class
	// TODO add comment
	public void detectBalls(Mat img) {
		List<MatOfPoint> contours = mDetector.findContours(img);
		Log.e(TAG, "found areas: " + contours.size());
		for (MatOfPoint area : contours) {

			List<MatOfPoint> ballArea = new ArrayList<MatOfPoint>();
			ballArea.add(area);

			Point center = computeCenterPt(ballArea);
			// Point pointGroundPlane = computePointGroundPlane();
			Point pointGroundPlane = null; // TODO implement (don't forget to
											// add the robot's pos coordinates)
			double rad = computeRadius3(ballArea, center);

			Ball detectedBall = new Ball(center, pointGroundPlane, rad);

			alignToBall(detectedBall);

			Core.circle(mRgbaOutput, center, 10, new Scalar(20), -1);
			Core.circle(mRgbaOutput, center, (int) rad, new Scalar(50), 5);
			Point lowestPoint = new Point(center.x, center.y + rad);
			Core.circle(mRgbaOutput, lowestPoint, 10, new Scalar(50), 5);

			// add only new balls to myBalls-list
			double TOL = 30;
			Point detectedBallPos = detectedBall.getPosGroundPlane(); // refactor
																		// variable
																		// name
			for (Ball b : foundBalls) {
				Point bPos = b.getPosGroundPlane();
				if (Math.abs(bPos.x - detectedBallPos.x) > TOL
						|| Math.abs(bPos.y - detectedBallPos.y) > TOL) {
					foundBalls.add(detectedBall);
					printBallInfo();
				} else {
					// TODO update radius etc.
				}
			}
		}
	}

	/**
	 * robot aligns his body to a surrendered Point
	 * 
	 * @param point
	 */
	public void alignToPoint(Point p) {
		Log.i(TAG, "(alignToPoint) p = " + p.toString());
		robot.writeLog("(alignToPoint) p = " + p.toString());
		Boolean aligned = false;
		double centerXAxis = mRgbaOutput.width() / 2;
		Log.i(TAG, "robot Camera xAxis: " + centerXAxis);
		double TOL = 200.0;
		double ballXAxis = p.x;
		while (!aligned) {
			ballXAxis = findCirclesOnCamera().get(0).x;
			Log.i(TAG, "ball xAxis: " + ballXAxis);
			double diff = centerXAxis - ballXAxis;
			Log.i(TAG, "axis difference:" + diff);
			if (Math.abs(diff) > TOL && diff < 0) {
				robot.turnRobot(5, 'r');
				Log.i(TAG, "(alignToPoint) turning right");
			} else if (Math.abs(diff) > TOL && diff > 0) {
				robot.turnRobot(2, 'l');
				Log.i(TAG, "(alignToPoint) turning left");
			} else {
				aligned = true;
			}
		}
		Log.i(TAG, "(alignToPoint) aligned");
		robot.writeLog("(alignToPoint) aligned");
	}

	/**
	 * 1) find ball 2) cage ball 3) move caged ball to target
	 */
	public void findAndDeliverBall() {
		Position finalPos = new Position(120, 120, 45);
		Log.i(TAG, "(findAndDeliverPoint) Start");
		robot.writeLog("(findAndDeliverPoint) Start");
		Ball myBall = detectOneBall();
		Log.i(TAG, "(findAndDeliverPoint) Ready to cage the ball");
		robot.writeLog("(findAndDeliverPoint) Ready to cage the ball");
		driveToBallAndCage(myBall, finalPos);
		Log.i(TAG, "(findAndDeliverPoint) Ball caged");
		robot.writeLog("Ball caged");
	}

	/**
	 * Turns robot for a maximum of 360°, stops when ball is adjusted to the
	 * center of the camera frame.
	 * 
	 * @return true if ball is found, false otherwise.
	 */
	public boolean turnAndFindABall() {

		Boolean foundBall = false;
		int turnedAngle = 0;
		List<Point> circles = new ArrayList<Point>();
		Log.i(TAG, "(turnAndFindABall) start");
		while (turnedAngle < 360 && !foundBall) {
			circles = findCirclesOnCamera();
			if (circles.size() > 0) {
				Log.i(TAG, "found circle x:" + circles.get(0).x + " y:"
						+ circles.get(0).y);
				alignToPoint(circles.get(0));
				Log.i(TAG, "(turnAndFindABall) found a ball");
				foundBall = true;
				break;
			}
			robot.turnRobot(5, 'r');
			turnedAngle += 5;
		}
		Log.i(TAG, "(turnAndFindABall) Finished");
		return foundBall;

	}

	/**
	 * Finds the centers of all circles on camera.
	 * 
	 * @return a list of centers of circles that are currently present on the
	 *         camera frame
	 */
	public List<Point> findCirclesOnCamera() {
		circleCenters = new ArrayList<Point>();
		Log.i(TAG,
				"(findCirclesOnCamera) Searching circles on camera; Number of colors: "
						+ myColors.size());
		for (Scalar hsvColor : myColors) {
			Mat grayImg = new Mat();
			grayImg = mDetector.filter(mRgbaWork, hsvColor);

			List<MatOfPoint> contours = mDetector.findContours(grayImg);
			Log.i(TAG,
					"(findCirclesOnCamera) Found following number of contours: "
							+ contours.size());

			for (MatOfPoint area : contours) {

				List<MatOfPoint> ballArea = new ArrayList<MatOfPoint>();
				ballArea.add(area);

				Point center = computeCenterPt(ballArea);

				circleCenters.add(center);
				Log.i(TAG, "(findCirclesOnCamera) Found circle on camera at: "
						+ center);
			}
		}

		Log.i(TAG,
				"(foundCirclesOnCamera) Found circles: " + circleCenters.size());

		return circleCenters;
	}

	/**
	 * Detects a ball.
	 * 
	 * @return Ball object if found, null otherwise.
	 */
	public Ball detectOneBall() {
		Log.i(TAG, "(detectOneBall) start");
		robot.writeLog("(detectOneBall) start");
		mRgbaWork = new Mat();
		mRgbaOutput.copyTo(mRgbaWork);
		Ball detectedBall = null;
		if (turnAndFindABall()) {
			Log.i(TAG, "(detectOneBall) Found ball");
			robot.writeLog("(detectOneBall) Found ball");
			for (Scalar hsvColor : myColors) {
				Mat grayImg = new Mat();
				grayImg = mDetector.filter(mRgbaWork, hsvColor);
				List<MatOfPoint> contours = mDetector.findContours(grayImg);
				Log.e(TAG, "found areas: " + contours.size());
				for (MatOfPoint area : contours) {

					List<MatOfPoint> ballArea = new ArrayList<MatOfPoint>();
					ballArea.add(area);

					Point center = computeCenterPt(ballArea);
					double rad = computeRadius3(ballArea, center);
					Point lowestPoint = new Point(center.x, center.y + rad);
					Point pointGroundPlane = getGroundPlaneCoordinates(lowestPoint);

					detectedBall = new Ball(center, pointGroundPlane, rad);
					Log.i(TAG,
							"(detectOneBall) found ball with following ground coordinates: "
									+ detectedBall.toString());
					robot.writeLog("(detectOneBall) found ball with following ground coordinates: "
									+ detectedBall.toString());
				}
			}
		}
		Log.i(TAG,
				"(detectOneBall) returning ball with following ground coordinates: "
						+ detectedBall.toString());
		robot.writeLog("(detectOneBall) returning ball with following ground coordinates: "
						+ detectedBall.toString());

		return detectedBall;
	}

	/**
	 * Drives to the ball and cages it.
	 * 
	 * @param ball
	 *            the ball to cage
	 */
	public void driveToBallAndCage(Ball ball, Position finalPos) {
		Log.i(TAG, "(driveToBallAndCage) start");
		robot.writeLog("(driveToBallAndCage) start");
		Point ballTarget = ball.getPosGroundPlane();
		Log.i(TAG,
				"(driveToBallAndCage) received groundPlane coordinates of ball: "
						+ ballTarget.toString());
		robot.writeLog("(driveToBallAndCage) received groundPlane coordinates of ball: "
						+ ballTarget.toString());
		Log.i(TAG,
				"(driveToBallAndCage) moving to ball: " + ballTarget.toString());
		robot.writeLog("(driveToBallAndCage) moving to ball: " + ballTarget.toString());
		robot.MoveToTarget(ballTarget.y, ballTarget.x, 0);
		Log.i(TAG, "(driveToBallAndCage) lowering bar");
		robot.writeLog("(driveToBallAndCage) lowering bar");
		robot.robotSetBar(0);
		double finalPosX = finalPos.x;
		double finalPosY = finalPos.y;
		double finalTheta = finalPos.theta;
		Log.i(TAG, "(driveToBallAndCage) move to final Position" + finalPos);
		robot.writeLog("(driveToBallAndCage) move to final Position" + finalPos);
		robot.MoveToTarget(finalPosY, finalPosX, finalTheta);
		robot.robotSetBar(120);
	}

	/**
	 * Takes a camera point and calculates its ground plane coordinates.
	 * 
	 * @param cameraPoint
	 * @return ground plane coordinates of camera point
	 */
	public Point getGroundPlaneCoordinates(Point cameraPoint) {

		Mat src = new Mat(1, 1, CvType.CV_32FC2);
		Mat dest = new Mat(1, 1, CvType.CV_32FC2);
		src.put(0, 0, new double[] { cameraPoint.x, cameraPoint.y }); // ps is a
																		// point
																		// in
																		// image
																		// coordinates
		Core.perspectiveTransform(src, dest, homographyMatrix); // homography is
																// your
																// homography
																// matrix
		Point pointGroundCoord = new Point(dest.get(0, 0)[0] / 10, dest.get(0,
				0)[1] / 10);
		Log.i(TAG,
				"(getGroundPlaneCoordinates) Found ground plane coordinates relative to robot: "
						+ pointGroundCoord.toString());
		double theta2 = Math.atan2(pointGroundCoord.x, pointGroundCoord.y);
		Log.i(TAG,
				"(getGroundPlaneCoordinates) Found ground plane coordinates relative to robot: "
						+ pointGroundCoord.toString());
		double dist = Math.sqrt(Math.pow(pointGroundCoord.x, 2)
				+ Math.pow(pointGroundCoord.y, 2));
		double dx = -dist*Math.sin(2*Math.PI - (theta2 + Math.toRadians(robot.getTg())));
		double dy = dist * Math.cos(2*Math.PI - (theta2 + Math.toRadians(robot.getTg())));

		Log.i(TAG,
				"(getGroundPlaneCoordinates) theta2: "
						+ theta2 + " theta: " + robot.getTg() + " dist: " + dist + " dx: " + dx + " dy: " + dy);
		
		pointGroundCoord.x = robot.getMyPosition().x + dx;
		pointGroundCoord.y = robot.getMyPosition().y + dy;
		
		Log.i(TAG,
				"(getGroundPlaneCoordinates) Found ground plane coordinates (global): "
						+ pointGroundCoord.toString());

		// TODO Add robot's current position
		return pointGroundCoord;
	}

	// TODO needed?
	// TODO if so, write comment
	private Scalar converScalarHsv2Rgba(Scalar hsvColor) {
		Mat pointMatRgba = new Mat();
		Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
		Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL,
				4);

		return new Scalar(pointMatRgba.get(0, 0));
	}

}
