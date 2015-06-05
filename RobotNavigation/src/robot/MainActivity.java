package robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.ReentrantReadWriteLock.WriteLock;

import jp.ksksue.driver.serial.FTDriver;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import robot.generated.R;
import robot.navigate.Position;
import robot.navigate.Robot;
import robot.opencv.ImageProcessor;
import robot.shapes.Ball;
import robot.shapes.Beacon;
import robot.shapes.BeaconSquareHolder;
import robot.shapes.Circle;
import robot.shapes.Square;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.view.WindowManager;
import android.widget.EditText;
import android.widget.ScrollView;
import android.widget.TextView;

public class MainActivity extends Activity implements OnTouchListener,
		CvCameraViewListener2 {

	// TODO: Use LEDs to display connection to the robot.

	// TODO: Probably add a function that allows to turn robot by velocity (and
	// since depending on the angle different correctorfactors are needed, we
	// need to do this via a switch case)

	// TODO: Add comments for variables.

	// TODO: Resolve warnings in all xml-files.

	// TODO: target Position does not allow negative inputs in GUI

	// TODO: Ex3: beacons: Detection of multiple, multi-colored objects, finding
	// their bottom points, calculating and displaying their locations in the
	// robot's egocentric ground-plane coordinates, as well as their distances
	// to the robot, using a pre-calibrated homography matrix

	// TODO: Do not allow the mobile phone to suspend

	// TODO: Ex3: The robot is placed at an arbitrary location within a
	// rectangular workspace of roughly 2.5m by 2.5m in size, surrounded by 8
	// beacons spaced 125cm apart and placed around the setup. Update the robot
	// odometry based on these values.

	// TODO: Ex3: About 10 balls of known colors are placed at arbitrary
	// locations within the workspace. Find them and bring to target position.

	// GUI Elements
	private TextView textLog; // Textview on GUI which contains the robot's log
	private EditText editText1; // Textfield on GUI for entering x-coordinate of
								// target
	private EditText editText2; // Textfield on GUI for entering y-coordinate of
								// target
	private EditText editText3; // Textfield on GUI for entering theta-alignment
								// at target

	// General variables
	private static final String TAG = "RobotLog"; // Tag for log-messages sent
													// to logcat
	private static final int CV_FONT_HERSHEY_COMPLEX = 0;

	// Used Classes
	private Robot robot; // Used to control the robot.
	private ImageProcessor imageProcessor; // Used for image processing.

	// Imageprocessing specific variables
	private Mat homographyMatrix; // the homography matrix is used to map camera
									// pixels to the ground plane (relative to
									// the robot's cam)
	private int frameInterval = 0; // Helper variable which is needed to
									// calculate the ball centers only every
									// 15th frame.
	private int executionInterval = 15; // Every executionInterval frames, the
	// objects are drawn into the camera
	// frame
	private Mat mRgbaOutput; // Current image for output (circles etc. can be
								// added to this image); updated every
								// cameraframe
	private Mat mRgbaWork; // Current image for image processing (not to be
							// modified!); updated every cameraframe

	private Integer onTouchOption = 0; // 0 -> read CircleColors; 1 -> read
										// BeaconColors

	private List<Scalar> myCircleColors = new ArrayList<Scalar>(); // Stores all
																	// currently
																	// recognized
																	// colors
																	// for balls
	private List<Scalar> myBeaconColors = new ArrayList<Scalar>(); // Stores all
																	// currently
																	// recognized
																	// colors
																	// for
																	// beacons

	private CameraBridgeViewBase mOpenCvCameraView; // interaction between
													// openCV and camera

	// TODO: write own method to update these lists
	List<Circle> circleList = new ArrayList<Circle>();

	List<Square> squareList = new ArrayList<Square>();
	List<Square> confirmedSquares = new ArrayList<Square>();

	List<Beacon> beaconList = new ArrayList<Beacon>();
	// Robot specific variables
	// TODO Use Position.java instead of targetX and targetY
	private double targetX = 100.0; // target's x-coordinate
	private double targetY = 100.0; // target's y-coordinate
	private int targetTheta = 45; // alignment at target point

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

		robot = new Robot(textLog, svLog, com, TAG, imageProcessor);
		robot.connect();

		mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
		mOpenCvCameraView.setCvCameraViewListener(this);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

		// initialize myBeaconColors & myColors
		myBeaconColors.add(new Scalar(18, 190, 180)); // orange
		myBeaconColors.add(new Scalar(132, 130, 135)); // blue // TODO:
														// recalibrate after
														// green was changed
		myBeaconColors.add(new Scalar(235, 170, 170)); // magenta
		myBeaconColors.add(new Scalar(100, 140, 115)); // green // TODO:
														// recalibrate after
														// green is changed.

		myCircleColors.add(new Scalar(100, 230, 170)); // green
		myCircleColors.add(new Scalar(252, 240, 190)); // red
		myCircleColors.add(new Scalar(144, 230, 150)); // blue
		myCircleColors.add(new Scalar(4, 210, 170)); // orange
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
				robot.moveByDistance(100);
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
					robot.turnByDistance(30, 'r');
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
				robot.turnByDistanceBalanced(90, 'r');
				robot.moveByVelocity(100, true);
				robot.turnByDistanceBalanced(135, 'l');
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

	public void ButtonExamination3(View v) {
		Thread t = new Thread() {

			@Override
			public void run() {
				collectAllBalls();
			};
		};

		t.start();
	}

	public void buttonTest_onClick(View v) {

		Thread t = new Thread() {

			@Override
			public void run() {
				robot.robotSetLeds(200, 200);
				robot.moveByVelocity(-130, false);
				while (!robot.updateGlobalPosition(findTwoBeacons(),
						homographyMatrix)) {
					robot.writeLog("Trying to update Position");
				}
				robot.robotSetLeds(0, 0);
			};
		};

		t.start();
	}

	public void buttonTest2_onClick(View v) {

		Thread t = new Thread() {

			@Override
			public void run() {
				Position targetPoint = new Position(targetX, targetY,
						targetTheta);
				robot.moveToTargetCollBalls(targetPoint, mRgbaWork,
						myCircleColors, homographyMatrix, confirmedSquares);
				;
			};
		};

		t.start();
	}

	public void buttonTest3_onClick(View v) {

		Thread t = new Thread() {

			@Override
			public void run() {

				robot.robotSetLeds(200, 200);
				// TODO: test this method
				// robot.driveToTargetCollectAllBalls(targetPoint, mRgbaWork,
				// myCircleColors, homographyMatrix, foundBalls);
				Ball nearestBall = robot.findNearestBall(mRgbaWork,
						myCircleColors, homographyMatrix, confirmedSquares);
				robot.robotSetLeds(0, 0);
				robot.writeLog(nearestBall.getPosGroundPlane().toString());
			};
		};

		t.start();
	}

	public void buttonTest4_onClick(View v) {

		Thread t = new Thread() {

			@Override
			public void run() {
				robot.robotSetLeds(200, 200);
				Position targetPoint = new Position(targetX, targetY,
						targetTheta);
				Ball nearestBall = robot.findNearestBall(mRgbaWork,
						myCircleColors, homographyMatrix, confirmedSquares);
				robot.robotSetLeds(0, 0);
				while (nearestBall != null) {
					robot.driveToBallAndCage2(nearestBall, mRgbaWork,
							myCircleColors, homographyMatrix, confirmedSquares);
					robot.writeLog("driving forward to next target");
					robot.moveToTargetWithoutAngle(targetX, targetY, 5);
					robot.writeLog("target point reached");
					robot.moveByVelocitySlow(-16, false);
					robot.turnByDistance(180, 'r');
					robot.writeLog("heading back to target point");
					try {
						Thread.sleep(1000);
					} catch (InterruptedException e) {
						// do nothing
					}
					robot.moveToTarget(targetPoint.x, targetPoint.y);
					robot.robotSetBar(260);
					robot.robotSetLeds(200, 200);
					robot.robotSetLeds(0, 0);
					// robot.updateGlobalPosition(findTwoBeacons(),
					// homographyMatrix);
					nearestBall = robot.findNearestBall(mRgbaWork,
							myCircleColors, homographyMatrix, confirmedSquares);
				}
				robot.robotSetLeds(100, 100);
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
						homographyMatrix = imageProcessor
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

		circleList = new ArrayList<Circle>();
		squareList = new ArrayList<Square>();
		confirmedSquares = new ArrayList<Square>();
		beaconList = new ArrayList<Beacon>();
		myCircleColors = new ArrayList<Scalar>();
		myBeaconColors = new ArrayList<Scalar>();
		robot.resetPosition();
		homographyMatrix = new Mat();
		textLog.setText("");

	}

	/**
	 * in the default case the OnTouch method adds a selected color to the
	 * circle list
	 * 
	 * this method allows you to switch the color target to the beacon list and
	 * vice versa
	 * 
	 * 
	 * @param v
	 */
	public void ButtontoggleInputColor(View v) {
		Thread t = new Thread() {

			@Override
			public void run() {
				if (onTouchOption == 0) {
					onTouchOption = 1; // read Beacon Color
					robot.writeLog("onTouch reads now beacon colors");
				} else {
					onTouchOption = 0; // read Circle Color
					robot.writeLog("onTouch reads now circle colors");
				}
			};
		};

		t.start();
	}

	public void ButtonFindAndDeliverBall(View v) {
		Thread t = new Thread() {

			@Override
			public void run() {
				robot.findAndDeliverBall(targetX, targetY, mRgbaWork,
						myCircleColors, homographyMatrix, confirmedSquares);
				robot.moveToTarget(0.0, 0.0, 0);
			};
		};

		t.start();
	}

	/**
	 * Enables access to the camera and on touch events on the according view.
	 */
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

	/**
	 * Closes camera view when application is closed or the system tries to free
	 * memory.
	 */
	public void onDestroy() {
		super.onDestroy();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	/**
	 * Initializes the camera view and associated image matrices with the given
	 * width and height.
	 * 
	 * @param width
	 *            width of the camera view (in pixels)
	 * @param height
	 *            height of the camera view (in pixels)
	 */
	public void onCameraViewStarted(int width, int height) {
		mRgbaWork = new Mat(height, width, CvType.CV_8UC4);
		mRgbaOutput = new Mat(height, width, CvType.CV_8UC4);
		imageProcessor = new ImageProcessor(TAG);
	}

	/**
	 * clears memory in case camera view is stopped
	 */
	public void onCameraViewStopped() {
		mRgbaWork.release();
		mRgbaOutput.release();
	}

	/**
	 * When camera view is touched, this method saves the color of the touched
	 * pixel (and averaged surrounding pixels). This color is then tracked for
	 * finding blobs in the view.
	 * 
	 * @param v
	 *            touched view
	 * @param event
	 *            object that is used to report movement (position etc.)
	 * @return false (no need for subsequent touch events)
	 */
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
		Scalar mBlobColorHsv = Core.sumElems(touchedRegionHsv);
		int pointCount = touchedRect.width * touchedRect.height;
		for (int i = 0; i < mBlobColorHsv.val.length; i++)
			mBlobColorHsv.val[i] /= pointCount;
		if (onTouchOption == 0) {
			myCircleColors.add(mBlobColorHsv);
		} else {
			myBeaconColors.add(mBlobColorHsv);
		}
		Log.i(TAG, "saved colors: " + myCircleColors.size() + mBlobColorHsv);
		robot.writeLog("saved colors: " + myCircleColors.size() + mBlobColorHsv);
		touchedRegionRgba.release();
		touchedRegionHsv.release();
		return false;
	}

	/**
	 * Gets called every camera frame. Updates the global image matrices
	 * mRgbaOutput and mRgbaWork which are used for image processing and image
	 * display.
	 * 
	 * @param inputFrame
	 *            current camera frame
	 * @return the image matrix to display
	 */
	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
		mRgbaOutput = inputFrame.rgba();
		mRgbaWork = inputFrame.rgba(); // TODO: does mRgbaWork refer to the same
										// image as mRgbaOutput? In that case,
										// either fix or remove this variable.
		if (frameInterval >= executionInterval) {
			squareList = imageProcessor.findSquaresOnCamera(mRgbaWork,
					myBeaconColors);
			BeaconSquareHolder beaconsAndSquares = imageProcessor
					.findBeacons(squareList);
			beaconList = beaconsAndSquares.getBeaconList();
			confirmedSquares = beaconsAndSquares.getSquareList();
			circleList = imageProcessor.findCirclesOnCamera2(mRgbaWork,
					myCircleColors, confirmedSquares);
			frameInterval = 0;
		}

		frameInterval++;

		// draw squares on CameraFrame

		// Mat grayImg = new Mat();
		// if (!myCircleColors.isEmpty()) {
		// for (Scalar s : myCircleColors)
		// grayImg = imageProcessor.filter(mRgbaWork, s, 'c');
		// mRgbaOutput = grayImg;
		// }

		for (Square s : squareList) {

			Point[] rect_points = new Point[4];
			s.points(rect_points);
			Core.circle(mRgbaOutput, s.getLowPt(), 10, new Scalar(255, 0, 0));
			Core.circle(mRgbaOutput, s.getHighPt(), 10, new Scalar(0, 0, 255));

			// Log.i(TAG, s.toString());
			for (int j = 0; j < 4; j++)
				Core.line(mRgbaOutput, rect_points[j],
						rect_points[(j + 1) % 4], new Scalar(0, 0, 0), 5);
			//
			// Core.rectangle(mRgbaOutput, s.getLowerLeftEdge(),
			// s.getUpperRightEdge(), new Scalar(20), -1);
			Core.putText(mRgbaOutput, "center", s.center,
					CV_FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 0, 255), 1, 8,
					false);
			// robot.writeLog(s.toString());
		}
		//
		// // draw confirmed squares
		for (Square s : confirmedSquares) {

			Point[] rect_points = new Point[4];
			s.points(rect_points);
			Core.circle(mRgbaOutput, s.getLowPt(), 10, new Scalar(255, 0, 0));
			Core.circle(mRgbaOutput, s.getHighPt(), 10, new Scalar(0, 0, 255));

			for (int j = 0; j < 4; j++)
				Core.line(mRgbaOutput, rect_points[j],
						rect_points[(j + 1) % 4], new Scalar(0, 0, 255), 5);
			//

			// Core.rectangle(mRgbaOutput, s.getLowerLeftEdge(),
			// s.getUpperRightEdge(), new Scalar(0, 0, 0), -1);
			// robot.writeLog(s.toString());
		}

		// draw circles on camera frame
		if (!circleList.isEmpty()) {
			for (Circle c : circleList) {
				Core.circle(mRgbaOutput, c.getCenter(), (int) c.getRadius(),
						new Scalar(160, 245, 5));
				Core.circle(mRgbaOutput, c.getLowPt(), (int) 10, new Scalar(0,
						0, 255));
			}
		}

		// draw Beacons
		if (!beaconList.isEmpty()) {
			for (Beacon b : beaconList) {
				Point[] rect_points = new Point[4];
				b.points(rect_points);

				for (int j = 0; j < 4; j++)
					Core.line(mRgbaOutput, rect_points[j],
							rect_points[(j + 1) % 4], new Scalar(240, 126, 12),
							5);
				//
				Core.putText(mRgbaOutput, "center", b.center,
						CV_FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 0, 255), 1,
						8, false);
				Core.circle(mRgbaOutput, b.center, 10, new Scalar(0));
				// Core.circle(mRgbaOutput, b.getLowerLeftEdge(), 10, new
				// Scalar(
				// 255, 0, 0));
				// Core.circle(mRgbaOutput, b.getUpperRightEdge(), 10, new
				// Scalar(
				// 255, 0, 0));
				Core.circle(mRgbaOutput, b.getLowPt(), 10,
						new Scalar(255, 0, 0));
				// Core.putText(mRgbaOutput, b.toString(), b.getLowerLeftEdge(),
				// CV_FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 0, 255), 1,
				// 8, false);
			}
		}

		frameInterval++;
		return mRgbaOutput;
	}

	/**
	 * Disables camera view if Application switches into background.
	 */
	@Override
	public void onPause() {
		super.onPause();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	/**
	 * Enables camera view if Application resumes.
	 */
	@Override
	public void onResume() {
		super.onResume();
		OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_3, this,
				mLoaderCallback);
	}

	/**
	 * Turns until two beacons are seen by the robot.
	 * 
	 * @return List of beacons.
	 */
	public List<Beacon> findTwoBeacons() {
		ImageProcessor imgProc = new ImageProcessor(TAG);
		List<Beacon> beacons = imgProc.findBeacons(confirmedSquares)
				.getBeaconList();
		int angle = 0;
		while (beacons.size() < 2 && angle < 360) {
			angle += 15;
			robot.turnByDistance(15, 'r');
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			beacons = imgProc.findBeacons(confirmedSquares).getBeaconList();
		}
		if (angle >= 360) {
			robot.moveByVelocity(30.0, false);
		}
		return beacons;
	}

	// TODO: test / implement
	/**
	 * 1)findTwoBeacons 2)drive to goal and cage ball one the way 3)after goal
	 * position reached -> search for one ball and bring it to the goal (repeat
	 * this procedure until all balls are at the target)
	 */
	public void collectAllBalls() {
		while (!robot.updateGlobalPosition(findTwoBeacons(), homographyMatrix)) {
			robot.writeLog("Trying to update Position");
		}
		robot.robotSetLeds(200, 200);
		Position targetPoint = new Position(targetX, targetY, targetTheta);
		robot.moveToTargetCollBalls(targetPoint, mRgbaWork, myBeaconColors,
				homographyMatrix, confirmedSquares);
		robot.robotSetLeds(0, 0);

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// do nothing
		}

		// TODO: test this method
		// robot.driveToTargetCollectAllBalls(targetPoint, mRgbaWork,
		// myCircleColors, homographyMatrix, foundBalls);
		robot.robotSetLeds(200, 200);
		Ball nearestBall = robot.findNearestBall(mRgbaWork, myCircleColors,
				homographyMatrix, confirmedSquares);
		robot.robotSetLeds(0, 0);
		while (nearestBall != null) {
			robot.driveToBallAndCage2(nearestBall, mRgbaWork, myCircleColors,
					homographyMatrix, confirmedSquares);
			robot.moveToTargetWithoutAngle(targetX, targetY, 5);
			robot.moveByVelocitySlow(-16, false);
			robot.turnByDistance(180, 'r');
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				// do nothing
			}
			robot.moveToTarget(targetPoint.x, targetPoint.y);
			robot.robotSetBar(300);
			robot.robotSetLeds(200, 200);
			robot.robotSetLeds(0, 0);
			robot.updateGlobalPosition(findTwoBeacons(), homographyMatrix);
			nearestBall = robot.findNearestBall(mRgbaWork, myCircleColors,
					homographyMatrix, confirmedSquares);
		}
		robot.robotSetLeds(100, 100);
	}

	/**
	 * updates global position using beacons every ~15
	 */
	public void UpdateselfLocalization() {
		findTwoBeacons();
		robot.updateGlobalPosition(beaconList, homographyMatrix);
	}

}
