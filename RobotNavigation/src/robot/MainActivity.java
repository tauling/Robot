package robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import robot.generated.R;
import robot.navigate.Ball;
import robot.navigate.Robot;
import robot.opencv.ColorBlobDetector;

import jp.ksksue.driver.serial.FTDriver;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.widget.EditText;
import android.widget.ScrollView;
import android.widget.TextView;

public class MainActivity extends Activity implements OnTouchListener,
		CvCameraViewListener2 {

	// TODO: Add a function that allows to drive curves (and updates odometry)

	// TODO: Probably add a function that allows to turns robot by velocity (and since depending on the angle different correctorfactors are needed, we need to do this via a switch case)

	// TODO: Explore workspace and remember positions of all balls

	// TODO: Add comments for variables.

	// TODO: Check if some methods should be moved to Robot.java or
	// ColorBlobDetection.java.

	// TODO: Resolve warnings in all xml-files.
	
	// TODO: target Position does not allow negative inputs in GUI

	private TextView textLog;

	private Robot robot;

	private Mat homographyMatrix;

	private EditText editText1;
	private EditText editText2;
	private EditText editText3;

	private double targetX = 100.0; // TODO Use Position.java

	private double targetY = 100.0;

	private Integer targetTheta = 45;
	private static final String TAG = "RobotLog";

	private Mat mRgbaOutput;
	private Mat mRgbaWork;
	private Scalar mBlobColorHsv;
	private ColorBlobDetector mDetector;

	private int frameInterval = 0; // Helper variable which is needed to
									// calculate the ball centers only every
									// 15th frame.

	private int executionInterval = 15; // Every executionInterval frames, the
										// objects are drawn into the camera
										// frame

	private List<Scalar> myColors = new ArrayList<Scalar>();

	private List<Ball> foundBalls = new ArrayList<Ball>(); // list which stores
															// all found balls

	List<Point> circleCenters = new ArrayList<Point>();

	private CameraBridgeViewBase mOpenCvCameraView;

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
		editText1 = (EditText) findViewById(R.id.editText1);
		editText2 = (EditText) findViewById(R.id.editText2);
		editText3 = (EditText) findViewById(R.id.editText3);
		textLog.setMovementMethod(new ScrollingMovementMethod());
		ScrollView svLog = (ScrollView) findViewById(R.id.scrollMe);

		FTDriver com = new FTDriver((UsbManager) getSystemService(USB_SERVICE));

		robot = new Robot(textLog, svLog, com);
		robot.connect();

		mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
		mOpenCvCameraView.setCvCameraViewListener(this);
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

	public void buttonOneMeterDriveByVelSlow_onClick(View v) {
		Thread t = new Thread() {

			@Override
			public void run() {
				robot.moveByVelocitySlow(100, false);
			};
		};

		t.start();
	}

	public void button360Deg_onClick(View v) {
		Thread t = new Thread() {

			@Override
			public void run() {
				for (int i = 0; i < 12; i++)
					robot.turnRobot(30, 'r');
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

	public void buttonReadTargetPoint(View v) {
		targetX = Integer.parseInt(editText1.getText().toString());
		targetY = Integer.parseInt(editText2.getText().toString());
		targetTheta = Integer.parseInt(editText3.getText().toString());
		robot.writeLog("new target at x: " + targetX + " y: " + targetY
				+ " theta: " + targetTheta);
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
				// Not needed currently
			};
		};

		t.start();
	}

	public void buttonTest2_onClick(View v) {

		Thread t = new Thread() {

			@Override
			public void run() {
				// Not needed currently
			};
		};

		t.start();
	}

	public void buttonTest3_onClick(View v) {

		Thread t = new Thread() {

			@Override
			public void run() {
				// Not needed currently
				robot.moveByVelocitySlow(100, false); // TODO Add a Button for
														// this method to
														// "Kalibrieriung"; Then
														// remove this line
														// here.
			};
		};

		t.start();
	}

	public void buttonTest4_onClick(View v) {

		Thread t = new Thread() {

			@Override
			public void run() {
				// not needed currently
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

	public void ButtonEmptyBrain(View v) {

		circleCenters = new ArrayList<Point>();
		myColors = new ArrayList<Scalar>();
		robot.resetPosition();
		homographyMatrix = new Mat();
		textLog.setText("");

	}

	public void ButtonFindAndDeliverBall(View v) {
		Thread t = new Thread() {

			@Override
			public void run() {
				findAndDeliverBall(targetX, targetY);
				robot.moveToTarget(0.0, 0.0, 0);
			};
		};

		t.start();
	}

	// TODO add comment
	private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
		@SuppressLint("ClickableViewAccessibility")
		@Override
		public void onManagerConnected(int status) {
			switch (status) {
			case LoaderCallbackInterface.SUCCESS: {
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

	// TODO add comment
	public void onDestroy() {
		super.onDestroy();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	// TODO add comment
	public void onCameraViewStarted(int width, int height) {
		mRgbaWork = new Mat(height, width, CvType.CV_8UC4);
		mRgbaOutput = new Mat(height, width, CvType.CV_8UC4);
		mDetector = new ColorBlobDetector();
		mBlobColorHsv = new Scalar(255);
	}

	// TODO add comment
	public void onCameraViewStopped() {
		mRgbaOutput.release();
	}

	// TODO add comment
	@SuppressLint("ClickableViewAccessibility")
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
		mBlobColorHsv = Core.sumElems(touchedRegionHsv);
		int pointCount = touchedRect.width * touchedRect.height;
		for (int i = 0; i < mBlobColorHsv.val.length; i++)
			mBlobColorHsv.val[i] /= pointCount;
		myColors.add(mBlobColorHsv);
		Log.i(TAG, "saved colors: " + myColors.size());
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
			return (new Point(-200.0, -200.0)); // TODO: Better to return null
												// in this case?
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

	// TODO add comment
	public double distPointToPoint(Point p1, Point p2) {
		return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
	}

	// TODO update comment
	/**
	 * measures distance from every point to center and computes average radius
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

	// TODO needed? (gets called inside detectBalls)
	// TODO if so, comment
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
				robot.turnRobotBalanced(15, 'r');
			} else if (Math.abs(diff) > TOL && diff < 0) {
				robot.turnRobotBalanced(10, 'l');
			}

		}
		return aligned;
	}

	// TODO add comment
	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
		mRgbaOutput = inputFrame.rgba();
		mRgbaWork = inputFrame.rgba(); // TODO: does mRgbaWork refer to the same
										// image as mRgbaOutput? In that case,
										// either fix or remove this variable.
		if (frameInterval >= executionInterval) {
			findCirclesOnCamera();
			frameInterval = 0;
		}
		if (!circleCenters.isEmpty()) {
			for (Point circleCenter : circleCenters)
				Core.circle(mRgbaOutput, circleCenter, 10, new Scalar(20), -1);
		}
		frameInterval++;

		return mRgbaOutput;
	}

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

	// TODO Needed?
	// TODO If so, comment
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

	// TODO: needed?
	// TODO: If so, move to colorblog-class
	// TODO add comment
	// TODO fix: this method is used to detect various balls; currently not
	// working
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
			double rad = computeRadius(ballArea, center);

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
		robot.writeLog("(alignToPoint) robot Camera xAxis: " + centerXAxis);
		double TOL = 150.0;
		double ballXAxis = p.x;
		while (!aligned) {
			ballXAxis = findCirclesOnCamera().get(0).x;
			Log.i(TAG, "ball xAxis: " + ballXAxis);
			double diff = centerXAxis - ballXAxis;
			Log.i(TAG, "axis difference:" + diff);
			if (Math.abs(diff) > TOL && diff < 0) {
				robot.turnRobotBalanced(30, 'r');
				Log.i(TAG, "(alignToPoint) turning right");
			} else if (Math.abs(diff) > TOL && diff > 0) {
				robot.turnRobotBalanced(25, 'l');
				Log.i(TAG, "(alignToPoint) turning left");
			} else {
				aligned = true;
			}
		}
		Log.i(TAG, "(alignToPoint) aligned");
		robot.writeLog("(alignToPoint) aligned");
	}

	// TODO update description
	/**
	 * 1) find ball 2) cage ball 3) move caged ball to target
	 */
	public void findAndDeliverBall(double x, double y) {
		Log.i(TAG, "(findAndDeliverPoint) Start");
		robot.writeLog("(findAndDeliverPoint) Start");
		Ball myBall = detectOneBall();
		Log.i(TAG, "(findAndDeliverPoint) Ready to cage the ball");
		robot.writeLog("(findAndDeliverPoint) Ready to cage the ball");
		if (myBall != null) {
			driveToBallAndCage(myBall);
			Log.i(TAG, "(findAndDeliverPoint) Ball caged");
			robot.writeLog("Ball caged");
			robot.moveToTargetWithoutAngle(x, y, 5);
			robot.moveByVelocitySlow(-16, false);
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				// do nothing
			}
			robot.robotSetBar(126);
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				// do nothing
			}
			robot.moveByVelocity(-35, false);
		}
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
			} else {
				robot.turnRobotBalanced(25, 'r');
				turnedAngle += 25;
			}
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
			Mat grayImg;
			do {
			grayImg = mDetector.filter(mRgbaWork, hsvColor);
			}
			while (grayImg.empty());
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
			grayImg.release();
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
		Ball detectedBall = null;
		if (turnAndFindABall()) {
			Log.i(TAG, "(detectOneBall) Found ball");
			robot.writeLog("(detectOneBall) Found ball");
			for (Scalar hsvColor : myColors) {
				Mat grayImg;
				do {
							grayImg = mDetector.filter(mRgbaWork, hsvColor);
				}
							while (grayImg.empty());
				List<MatOfPoint> contours = mDetector.findContours(grayImg);
				Log.e(TAG, "found areas: " + contours.size());
				for (MatOfPoint area : contours) {

					List<MatOfPoint> ballArea = new ArrayList<MatOfPoint>();
					ballArea.add(area);

					Point center = computeCenterPt(ballArea);
					double rad = computeRadius(ballArea, center);
					Point lowestPoint = new Point(center.x, center.y + rad);
					Point pointGroundPlane = getGroundPlaneCoordinates(lowestPoint);

					detectedBall = new Ball(center, pointGroundPlane, rad);
					Log.i(TAG,
							"(detectOneBall) found ball with following ground coordinates: "
									+ detectedBall.toString());
					robot.writeLog("(detectOneBall) found ball with following ground coordinates: "
							+ detectedBall.toString());
				}
				grayImg.release();
			}
			Log.i(TAG,
					"(detectOneBall) returning ball with following ground coordinates: "
							+ detectedBall.toString());
			robot.writeLog("(detectOneBall) returning ball with following ground coordinates: "
					+ detectedBall.toString());
		}

		return detectedBall;
	}

	/**
	 * Drives to the ball and cages it.
	 * 
	 * @param ball
	 *            the ball to cage
	 */
	public void driveToBallAndCage(Ball ball) {
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
		robot.writeLog("(driveToBallAndCage) moving to ball: "
				+ ballTarget.toString());
		robot.moveToTargetWithoutAngle(ballTarget.x, ballTarget.y, 25);
		ballTarget = detectOneBall().getPosGroundPlane();
		robot.writeLog("readjusting");
		robot.moveToTargetWithoutAngle(ballTarget.x, ballTarget.y, 5);
		Log.i(TAG, "(driveToBallAndCage) lowering bar");
		robot.writeLog("(driveToBallAndCage) lowering bar");
		robot.robotSetBar(50);
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
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
		double dx = -dist
				* Math.sin(2 * Math.PI
						- (theta2 + Math.toRadians(robot.getTg())));
		double dy = dist
				* Math.cos(2 * Math.PI
						- (theta2 + Math.toRadians(robot.getTg())));

		Log.i(TAG, "(getGroundPlaneCoordinates) theta2: " + theta2 + " theta: "
				+ robot.getTg() + " dist: " + dist + " dx: " + dx + " dy: "
				+ dy);

		pointGroundCoord.x = robot.getMyPosition().x + dx;
		pointGroundCoord.y = robot.getMyPosition().y + dy;

		Log.i(TAG,
				"(getGroundPlaneCoordinates) Found ground plane coordinates (global): "
						+ pointGroundCoord.toString());

		src.release();
		dest.release();
		return pointGroundCoord;
	}

}
