package robot.navigate;

import java.util.HashMap;
import java.util.Map;

import robot.generated.R;

import jp.ksksue.driver.serial.FTDriver;
import android.app.Activity;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.os.Handler;
import android.text.method.ScrollingMovementMethod;
import android.view.View;
import android.widget.ScrollView;
import android.widget.TextView;

public class MainActivity extends Activity {

	// TODO: Add button to stop all threads

	// TODO: Fix functions which are listed under "on Development" (see GUI)

	// TODO: Allow the user to enter values in the app.

	// TODO: Add a function that allows to drive curves (and updates odometry)

	// TODO: Add a function that allows to drive an eight using curves.

	// TODO: Add a button, that demonstrates that the robot stops when there is
	// an obstacle on
	// the way while performing the eight test

	// TODO: Add a button, that demonstrates that the robot is performing the
	// eight test including
	// obstacle avoidance.

	// TODO: Detect green and red blobs

	// TODO: Detect a ball and calculate it's lowest position (where it touches
	// the surface)

	// TODO: Move to the ball and use the robot's cage to catch it.

	// TODO: Detect multiple balls at the same time

	// TODO: Also detect blue, yellow, black and white blobs

	private int balancedAngle = 0;
	private int balancedDist = 0;

	private TextView textLog;
	private ScrollView svLog;
	private FTDriver com;
	private Handler mhandler = new Handler();

	private class WriteLogRunnable implements Runnable {

		private String text = null;

		public WriteLogRunnable(String text) {
			this.text = text;
		}

		public void run() {

			mhandler.post(new Runnable() {
				public void run() {

					textLog.append(text);
					svLog.fullScroll(ScrollView.FOCUS_DOWN);
				}
			});
		}

	}

	// -> Robot Calibration
	private Integer ObsDetectBorderLR = 20; // Measurements in front of the
											// robot
											// below this value are treated as
											// obstacle (Working range of
											// left/right
											// sensor is 10 to 80cm)
	private Integer ObsDetectBorderM = 20; // Measurements below this value are
											// treated as obstacle (Working
											// range of middle sensor is 20 to
											// 80cm)
	private Integer ObsDetectBorderL = 20; // Measurements to the left of the
											// robot
	// below this value are treated as
	// obstacle (Working range of left
	// sensor is 10 to 80cm)
	private Integer ObsDetectBorderR = 20; // Measurements to the right of the
											// robot
	// below this value are treated as
	// obstacle (Working range of right
	// sensor is 10 to 80cm)
	private double CorrFactMoveForwardByDist = 5875.0 / 4309.0; // Should be
																// set,
																// such that
	// MoveRobot(100) moves
	// the
	// the robot for 100cm.

	private double CorrFactMoveForwardByVel = 303.0 / 650.0;
	private double CorrFactAngle = 8.0 / 7; // Should be set, such that
											// turnRobot(360)
											// rotates for exactly 360 degrees.
	private final int IdSensorLeft = 7; // Call findSensorIDs() to determine the
										// corresponding ID
	private final int IdSensorRight = 8; // Call findSensorIDs() to determine
											// the corresponding ID
	private final int IdSensorMiddle = 9; // Call findSensorIDs() to determine
											// the corresponding ID

	private int OffsetSensorLeft = 0; // Should be set, such that the left
										// sensor
	// measures distances correctly in its working
	// range.
	private int OffsetSensorRight = -1; // Should be set, such that the right
										// sensor
	// measures distances correctly in its working
	// range.
	private int OffsetSensorMiddle = 0; // Should be set, such that the left
										// sensor
	// measures distances correctly in its working
	// range.
	private int RobotLength = 18; // Should be set to the length of the robot in
									// cm.
	private int DistToPassObstacleL = RobotLength + ObsDetectBorderL + 3; // Distance
																			// to
																			// move
																			// once
																			// obstacleLeft()
																			// says
																			// false
																			// to
																			// pass
																			// the
																			// previously
																			// found
																			// obstacle.

	// <- Robot Calibration

	private double Xg = 0, Yg = 0; // Position of the robot.
	private int Tg = 0; // Angle of the robot.

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

		svLog = (ScrollView) findViewById(R.id.scrollMe);

		textLog = (TextView) findViewById(R.id.textLog);
		textLog.setMovementMethod(new ScrollingMovementMethod());

		com = new FTDriver((UsbManager) getSystemService(USB_SERVICE));
		connect();
	}

	public void robotSetBar(int value) {
		if (Math.abs(value) > 127) {
			value = (int) (Math.signum(value) * 127);
		}
		comReadWrite(new byte[] { 'o', (byte) value, '\r', '\n' });
	}

	/**
	 * Establishes connection to the robot.
	 */
	public void connect() {
		if (com.begin(FTDriver.BAUD9600)) {
			writeLog("connected");
		} else {
			writeLog("could not connect");
		}
	}

	/**
	 * Closes the connection to the robot.
	 */
	public void disconnect() {
		com.end();
		if (!com.isConnected()) {
			writeLog("disconnected");
		}
	}

	/**
	 * transfers given bytes via the serial connection.
	 * 
	 * @param data
	 */
	public void comWrite(byte[] data) {
		if (com.isConnected()) {
			com.write(data);
		} else {
			writeLog("not connected");
		}
	}

	/**
	 * Reads from the serial buffer. Due to buffering, the read command is
	 * issued 3 times at minimum and continuously as long as there are bytes to
	 * read from the buffer. Note that this function does not block, it might
	 * return an empty string if no bytes have been read at all.
	 * 
	 * @return buffer content as string
	 */
	public String comRead() {
		String s = "";
		if (com.isConnected()) {
			int n;
			byte[] buffer = new byte[256];
			n = com.read(buffer);
			s += new String(buffer, 0, n);
		} else {
			writeLog("not connected");
		}
		return s;
	}

	/**
	 * Write data to serial interface, wait 300 ms and read answer.
	 * 
	 * @param data
	 *            to write
	 * @return answer from serial interface
	 */
	public String comReadWrite(byte[] data) {
		return comReadWrite(data, 300);
	}

	/**
	 * Write data to serial interface, wait for the specified time and read
	 * answer.
	 * 
	 * @param data
	 *            to write
	 * @param time
	 *            to wait in ms
	 * @return answer from serial interface
	 */
	public String comReadWrite(byte[] data, int time) {
		comWrite(data);
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
		}
		return comRead();
	}

	/**
	 * Add the given text to the log file; also writes the length of the text
	 * into the log-file.
	 * 
	 * @param text
	 */
	public void writeLog(String text) {
		if (!(text == null) && text.length() > 0) {
			new Thread(new WriteLogRunnable("[" + text.length() + "] " + text
					+ "\n")).start();
			// mhandler.post(new MyRunnable("[" + text.length() + "] " + text
			// + "\n"));
		}
	}

	/**
	 * Add the given integer to the log file.
	 * 
	 * @param value
	 */
	public void writeLog(int value) {
		new Thread(new WriteLogRunnable(value + "\n")).start();
	}

	/**
	 * Add the given long integer to the log file.
	 * 
	 * @param value
	 */
	public void writeLog(long value) {
		new Thread(new WriteLogRunnable(value + "\n")).start();
	}

	/**
	 * Sets the blue and red LED of the robot.
	 * 
	 * @param red
	 *            sets the intensity of the red colored LED (min value 0, max
	 *            value 127)
	 * @param blue
	 *            sets the intensity of the blue colored LED (min value 0, max
	 *            value 127)
	 */
	public void robotSetLeds(int red, int blue) {
		writeLog(comReadWrite(new byte[] { 'u',
				(byte) Math.max(Math.min(red, 127), 0),
				(byte) Math.max(Math.min(blue, 127), 0), '\r', '\n' }));
	}

	public void buttonMinus_onClick(View v) {
		writeLog("Called by Minus");
		writeLog(comReadWrite(new byte[] { '-', '\r', '\n' }));
	}

	public void buttonPlus_onClick(View v) {
		writeLog("Called by Plus");
		writeLog(comReadWrite(new byte[] { '+', '\r', '\n' }));
	}

	public void buttonSensor_onClick(View v) {
		Map<String, Integer> measurement = new HashMap<String, Integer>();
		measurement = getDistance();
		for (Map.Entry<String, Integer> entry : measurement.entrySet()) {
			writeLog(entry.getKey() + entry.getValue());
		}
	}

	public void buttonEight_onClick(View v) {
		moveSquare(50, 'r', 0);
		turnRobot(90, 'l');
		moveSquare(50, 'l', 0);
	}

	public void buttonDriveAndRead_onClick(View v) {
		Thread t = new Thread() {

			@Override
			public void run() {
				driveAndRead();
			};
		};

		t.start();
	}

	public void buttonBug1_onClick(View v) {

		Thread t = new Thread() {

			@Override
			public void run() {
				bug1(200, 200);
			};
		};

		t.start();
	}

	public void buttonTest_onClick(View v) {

		Thread t = new Thread() {

			@Override
			public void run() {
				turnAndCheckRightSide();
			};
		};

		t.start();
	}

	public void buttonTest2_onClick(View v) {

		Thread t = new Thread() {

			@Override
			public void run() {
				bug1(100, 100);
			};
		};

		t.start();
	}

	public void buttonMoveToGoalN3_onClick(View v) {
		Thread t = new Thread() {

			@Override
			public void run() {
				moveToGoalNaive3(200, 200, 45);
			};
		};

		t.start();
	}

	public void buttonBug2_onClick(View v) {
		Thread t = new Thread() {

			@Override
			public void run() {
				bug2(100, 100);
			};
		};

		t.start();
	}

	public void buttonFindSensorIDs_onClick(View v) {
		try {
			findSensorIDs();
		} catch (Exception e) {
		}
	}

	public void buttonOneMeter_onClick(View v) {
		Thread t = new Thread() {

			@Override
			public void run() {
				moveRobot(100);
			};
		};

		t.start();
	}

	public void buttonOneMeterDriveByVel_onClick(View v) {
		Thread t = new Thread() {

			@Override
			public void run() {
				moveByVelocity(100, false);
			};
		};

		t.start();
	}

	public void button360Deg_onClick(View v) {
		Thread t = new Thread() {

			@Override
			public void run() {
				turnRobot(180, 'r');
				turnRobot(180, 'r');
			};
		};

		t.start();
	}

	public void buttonrotateToWall_onClick(View v) {
		Thread t = new Thread() {

			@Override
			public void run() {
				rotateToWall();
			};
		};

		t.start();
	}

	/**
	 * Drive by velocity, update position afterwards.
	 * 
	 * @param dist
	 *            distance to drive [cm]
	 * @param stopOnObstacle
	 *            when true, stop before hitting obstacle
	 * @return true when there is no obstacle in front, false otherwise
	 */
	public Boolean moveByVelocity(double dist, boolean stopOnObstacle) {
		double start = System.currentTimeMillis(); // [ms]
		double curTime = start;
		int velocity = 15;
		writeLog("startTime: " + (int) start);
		double speed = (100.0 / (1000 * velocity)) / CorrFactMoveForwardByVel; // [cm/ms]
		double corrTime = dist / speed; // [ms]
		double end = start + corrTime;
		boolean freeWay = true;
		comReadWrite(new byte[] { 'i', (byte) velocity, (byte) velocity, '\r',
				'\n' });
		while (curTime <= end && (freeWay || !stopOnObstacle)) {
			if (obstacleInFront()) {
				freeWay = false;
			}
			curTime = System.currentTimeMillis();
		}
		comReadWrite(new byte[] { 'i', (byte) 0, (byte) 0, '\r', '\n' });
		double movedTime = curTime - start;

		updatePosition((int) (movedTime * speed));
		return freeWay;
	}

	/**
	 * updates global Position parameters after Robot moved one stepLength
	 * 
	 * @param stepLength
	 */
	public void updatePosition(int stepLength) {
		double movementX = 0, movementY = 0;
		movementX = Math.cos(Math.toRadians((double) Tg)) * stepLength;
		movementY = Math.sin(Math.toRadians((double) Tg)) * stepLength;
		Xg += movementX;
		Yg += movementY;
		writeLog("my Position: (" + Xg + "," + Yg + "," + Tg + ")");
	}

	public int getAngleToGoal(double x, double y) {
		int angle = (int) (Math.toDegrees(Math.atan2(y - Yg, x - Xg)));

		return reduceAngle(angle - Tg);
	}
	
	public int getDistanceToGoal(double x, double y) {
		return (int) Math.sqrt(Math.pow(x - Xg, 2) + Math.pow(y - Yg, 2));
	}

	public int reduceAngle(int angle) {
		if (angle < 0) {
			angle += 360;
		}
		if (angle >= 360) {
			angle -= 360;
		}

		return angle;
	}

	/**
	 * update the robots own position information (only angle)
	 * 
	 * @param angle
	 * @param turn
	 *            direction
	 */
	public void updateRotation(int angle, char dir) {
		switch (dir) {
		case 'l':
			Tg -= angle;
			break;
		case 'r':
			Tg += angle;
			break;
		default:
			System.out.println("wrong input direction");
			break;
		}
		Tg = reduceAngle(Tg);
		writeLog("my Position: (" + Xg + "," + Yg + "," + Tg + ")");
	}

	/**
	 * move robot specific distance
	 * 
	 * @param dist
	 */
	public void moveRobot(int dist) {
		int corrDist = (int) (dist * CorrFactMoveForwardByDist);
		int waitTimeFact = 100;
		while (Math.abs(corrDist) > 127) { // Byte stores values from -128 to
											// 127
			corrDist -= (int) (Math.signum(corrDist)) * 127;
			writeLog(comReadWrite(
					new byte[] { 'k',
							(byte) ((int) (Math.signum(corrDist)) * 127), '\r',
							'\n' }, (int) (127 / CorrFactMoveForwardByDist)
							* waitTimeFact));
		}
		writeLog(comReadWrite(new byte[] { 'k', (byte) corrDist, '\r', '\n' },
				(int) Math.abs(corrDist / CorrFactMoveForwardByDist)
						* waitTimeFact));
		updatePosition(dist);
	}

	/**
	 * Moves forward until a) max_dist is reached; b) an obstacle is found; c)
	 * robot is on m-line again
	 * 
	 * @param max_dist
	 *            maximum distance [cm] to drive
	 * @param goal_x
	 *            final goal (x-coordinate) of the robot
	 * @param goal_y
	 *            final goal (y-coordinate) of the robot
	 * @return two booleans; the first is true when the robot is placed on the
	 *         m-line, the second one is true, when there was an obstacle on the
	 *         way
	 */
	public Boolean[] driveToIntersectionMLine(int max_dist, int goal_x,
			int goal_y) {
		double x_ofIntersection = (Yg - (Xg * Math.tan(Math.toRadians(Tg))))
				/ ((((double) goal_y) / goal_x) - (Math.tan(Math.toRadians(Tg))));
		double y_ofIntersection = x_ofIntersection * ((double) goal_y) / goal_x;
		double dist = Math.min(getDistanceToGoal(x_ofIntersection,y_ofIntersection), max_dist);

		Boolean[] ret = new Boolean[2];

		ret[1] = moveByVelocity(dist, true);

		if (ret[1] && (dist < max_dist)) {
			ret[0] = true;
		} else {
			ret[0] = false;
		}

		return ret;
	}

	// TODO: Add Buttons to demonstrate
	/**
	 * tells the robot to move along a square
	 * 
	 * @param dir
	 *            ("l" = left; "r" = right)
	 * @param dist
	 *            in cm
	 * @param obstacleTreatment
	 *            0 = ignore obstacle; 1 = stop before obstacle; 2 = navigate
	 *            around obstacle
	 */
	public void moveSquare(int dist, char dir, int obstacleTreatment) {
		switch (obstacleTreatment) {
		case 0:

			for (int i = 0; i < 4; i++) {
				turnRobotBalanced(90, dir);
				moveByVelocity(dist, false);
			}
			moveToGoalNaive3(0, 0, 0);
			break;
		case 1:
			turnRobotBalanced(90, dir);
			for (int i = 0; i < 4; i++) {
				if (moveByVelocity(dist, true)) {
					turnRobotBalanced(90, dir);
				}
			}
			moveToGoalNaive3(0, 0, 0);
			break;
		case 2:
			switch (dir) {
			case 'r':
				moveToGoalNaive3(dist, 0, 90);
				moveToGoalNaive3(dist, dist, 0);
				moveToGoalNaive3(0, dist, 270);
				moveToGoalNaive3(0, 180, 0);
				break;
			case 'l':
				moveToGoalNaive3(-dist, 0, 270);
				moveToGoalNaive3(-dist, -dist, 180);
				moveToGoalNaive3(0, -dist, 90);
				moveToGoalNaive3(0, 0, 0);
				break;
			}
		}
	}

	/**
	 * Tells the robot to turn
	 * 
	 * @param angle
	 *            in degrees
	 * @param dir
	 *            ("l" = left; "r" = right)
	 */
	public void turnRobotBalanced(int angle, char dir) {
		angle = reduceAngle(angle);
		switch (dir) {
		case 'l':
			if (Math.abs(balancedAngle - angle) < Math.abs(balancedAngle
					+ (360 - angle))) {
				balancedAngle -= angle;
				turnRobot(angle, 'l');
			} else {
				balancedAngle += (360 - angle);
				turnRobot(360 - angle, 'r');
			}
			break;
		case 'r':
			if (Math.abs(balancedAngle + angle) < Math.abs(balancedAngle
					- (360 - angle))) {
				balancedAngle += angle;
				turnRobot(angle, 'r');
			} else {
				balancedAngle -= (360 - angle);
				turnRobot(360 - angle, 'l');
			}

		}
	}

	/**
	 * Tells the robot to turn
	 * 
	 * @param angle
	 *            in degrees
	 * @param dir
	 *            ("l" = left; "r" = right)
	 */
	public void turnRobot(int angle, char dir) {
		int waitTimeFact = 17;
		angle = reduceAngle(angle);
		updateRotation(angle, dir);
		int degrees = angle;

		degrees = (int) (CorrFactAngle * degrees);

		switch (dir) {
		case 'r':
			degrees = -degrees;
			break;
		}

		while (Math.abs(degrees) > 127) { // Byte stores values from -128 to
											// 127
			degrees -= (int) (Math.signum(degrees)) * 127;
			writeLog(comReadWrite(
					new byte[] { 'l',
							(byte) ((int) (Math.signum(degrees)) * 127), '\r',
							'\n' }, waitTimeFact * 127));
		}
		writeLog(comReadWrite(new byte[] { 'l', (byte) degrees, '\r', '\n' },
				waitTimeFact * Math.abs(degrees)));
	}

	/**
	 * returns values of all sensors including ID. The ID is then used to
	 * associate the available physical sensors with the ID
	 */
	public void findSensorIDs() {
		int sensNr;
		int val;
		String[] sensorInfo;
		sensorInfo = comReadWrite(new byte[] { 'q', '\r', '\n' }).split(" ");
		sensNr = 1;
		for (String value : sensorInfo) {
			try {
				val = Integer.parseInt(value.substring(2, 4), 16);
				writeLog(sensNr + ": " + val);
				sensNr++;
			} catch (NumberFormatException e) {
				// Can't be parsed; do nothing
			}
		}
	}

	/**
	 * returns distances of all sensors in cm
	 */
	public Map<String, Integer> getDistance() {
		int sensNr;
		int val;
		String[] sensorInfo;
		Map<String, Integer> readSensor = new HashMap<String, Integer>();
		sensorInfo = comReadWrite(new byte[] { 'q', '\r', '\n' }).split(" ");
		sensNr = 1;
		for (String value : sensorInfo) {
			try {
				val = Integer.parseInt(value.substring(2, 4), 16);

				switch (sensNr) {
				case IdSensorLeft:
					readSensor.put("frontLeft", val + OffsetSensorLeft);
					break;
				case IdSensorRight:
					readSensor.put("frontRight", val / 2 + OffsetSensorRight);
					break;
				case IdSensorMiddle:
					readSensor.put("frontMiddle", val + OffsetSensorMiddle); // Middle
																				// Sensor
																				// quite
					// exact down to 20cm.
					// Below 20cm sensor
					// output increases
					// again.
					break;
				}
				sensNr++;
			} catch (NumberFormatException e) {
				// Can't be parsed; do nothing
			}
		}
		return readSensor;
	}

	/**
	 * Obstacle avoidance
	 * 
	 * drive on straight line and read sensor (check for obstacles)
	 */
	public void driveAndRead() {
		int i = 0;
		comReadWrite(new byte[] { 'i', 15, 15, '\r', '\n' });
		while (i < 3) {
			try {
				Thread.sleep(50);
			} catch (Exception e) {
			}
			if (obstacleInFront()) { // checks
										// if
										// robot
										// hit
										// near
				// obstacle
				comReadWrite(new byte[] { 'i', 0, 0, '\r', '\n' });
				writeLog(comReadWrite(new byte[] { 'u', (byte) 127, (byte) 0,
						'\r', '\n' }));
				turnRobot(90, 'l');
				try {
					Thread.sleep(1000);
				} catch (Exception e) {
				}
				writeLog(comReadWrite(new byte[] { 'u', (byte) 0, (byte) 127,
						'\r', '\n' }));
				comReadWrite(new byte[] { 'i', 15, 15, '\r', '\n' });
				i++;
			}
			writeLog(getDistance().get("frontMiddle"));
		}
		comReadWrite(new byte[] { 'i', 0, 0, '\r', '\n' });
	}

	/**
	 * methods tests whether an obstacle is in the range of the 3 front sensors
	 * or not
	 * 
	 * @return true - obstacle detected; false - free way
	 */
	public Boolean obstacleInFront() {
		Boolean detected = false;
		if (com.isConnected()) {
			Map<String, Integer> measurement = new HashMap<String, Integer>();
			measurement = getDistance();
			if (measurement.get("frontLeft") <= ObsDetectBorderLR
					|| measurement.get("frontRight") <= ObsDetectBorderLR
					|| measurement.get("frontMiddle") <= ObsDetectBorderM) {
				detected = true;
			}
		} else {
			detected = true;
			writeLog("No connection to the robot. Sensors can't be read.");
		}
		return detected;
	}

	/**
	 * checks if an obstacle is in the range of the left sensor
	 * 
	 * @return
	 */
	public Boolean obstacleLeft() {
		Boolean detected = false;
		if (com.isConnected()) {
			Map<String, Integer> measurement = new HashMap<String, Integer>();
			measurement = getDistance();
			if (measurement.get("frontLeft") <= ObsDetectBorderL) {
				detected = true;
			}
		} else {
			detected = true;
			writeLog("No connection to the robot. Sensors can't be read.");
		}
		return detected;
	}

	/**
	 * checks if an obstacle is in the range of the right sensor
	 * 
	 * @return
	 */
	public Boolean obstacleRight() {
		Boolean detected = false;
		Map<String, Integer> measurement = new HashMap<String, Integer>();
		if (com.isConnected()) {
			measurement = getDistance();
			if (measurement.get("frontRight") <= ObsDetectBorderR) {
				detected = true;
			}
		} else {
			detected = true;
			writeLog("No connection to the robot. Sensors can't be read.");
		}
		return detected;
	}

	/**
	 * Bug1 algorithm 1) head toward goal 2) if an obstacle is encountered
	 * circumnavigate it and remember how close you get to the goal 3) return to
	 * that closest point(by wall-following) and continue
	 */
	public void bug1(int x, int y) {
		int dist;
		int angle;
		int moved = 0;

		angle = getAngleToGoal(x,y);
		dist = getDistanceToGoal(x,y);

		writeLog("Moving to goal at angle " + angle + " in " + dist
				+ "cm distance");

		// we need to update the robots own position information
		turnRobot(angle, 'r');

		while (moved < dist) {
			moved++;
			moveByVelocity(2, false);
			if (obstacleInFront()) {
				writeLog("Obstacle found at " + getMyPosition());
				moveByVelocity(15, false);
				roundObstacle(x, y);
				break;
			}

		}
	}

	/**
	 * turn 90 deg left check for obstacle turn 90 deg right
	 * 
	 * @return whether an obstacle is on the left side
	 */
	public Boolean turnAndCheckObstacle() {
		boolean detected = false;
		turnRobot(90, 'l');
		if (obstacleInFront()) {
			detected = true;
		}
		turnRobot(90, 'r');
		return detected;
	}

	/**
	 * turn Robot a bit to obstacle and measure distance with left sensor before
	 * and after rotation if the distance gets smaller the robot should rotate
	 * back more than he turn in, otherwise (the distance increases) the robot
	 * needs to turn a lot more to the right
	 * 
	 * @return whether an obstacle is on the left side
	 */
	public Boolean turnAndCheckRightSide() {
		boolean detected = false;
		Map<String, Integer> measurement = new HashMap<String, Integer>();
		measurement = getDistance();
		int startDistLeft = measurement.get("frontLeft");
		int startDistRight = measurement.get("frontRight");
		writeLog("distance left: " + startDistLeft + "; right: "
				+ startDistRight);
		int deg = 20;
		writeLog("turn robot 20° to obstacle");
		turnRobot(deg, 'l');
		measurement = getDistance();
		int currDistLeft = measurement.get("frontLeft");
		int currDistRight = measurement.get("frontRight");
		if (currDistLeft < 12) {
			writeLog("drive back");
			moveRobot(-10);
		}
		if (currDistRight < 12) {
			moveRobot(-10);
			writeLog("drive back + rotate 45° right");
			turnRobot(45, 'r');
			currDistLeft = measurement.get("frontLeft");
			currDistRight = measurement.get("frontRight");
		}
		writeLog("distance left: " + currDistLeft + "; right: " + currDistRight);
		int diffL = currDistLeft - startDistLeft;
		int diffR = currDistRight - startDistRight;
		writeLog("difference start and current measurment(Left): " + diffL);
		writeLog("difference start and current measurment(Right): " + diffR);
		int TOL = 0;
		int turnAngle = deg + 15;
		if (Math.abs(diffL) > TOL && diffL < 0) {
			writeLog("turn " + turnAngle + "°");
			turnRobot(turnAngle, 'r');
		} else if (Math.abs(diffR) > TOL && diffR > 0 && currDistRight < 80) {
			writeLog("turn " + 90 + "°");
			turnRobot(90, 'r');
		} else if (Math.abs(diffR) > TOL && diffR < 0 && currDistLeft > 100) {
			writeLog("turn " + turnAngle + "°");
			turnRobot(turnAngle, 'r');
		} else if (Math.abs(diffR) - Math.abs(diffL) < TOL) {
			turnRobot(45, 'r');
		}
		if (obstacleInFront()) {
			detected = true;
		}
		return detected;
	}

	/**
	 * Uses move robot by distance to drive to a given goal destination while circumventing obstacles
	 * by rotating by a random angle and moving a couple of centimeters everytime an obstacle is found
	 * @param x the goal's x-coordinate
	 * @param y the goal's y-coordinate
	 * @param theta once the robot reaches the goal destination it should rotate to be in this angle
	 */
	public void moveToGoalNaive2(double x, double y, int theta) {
		int dist;
		int angle = 0;
		int moved;
		int stepLength = 5;
		boolean obstacleFound;
		boolean goalReached = false;
		Map<String, Integer> measurement = new HashMap<String, Integer>();

		while (!goalReached) {
			obstacleFound = false;

			angle = getAngleToGoal(x, y);
			dist = (int) Math.sqrt(Math.pow(x - Xg, 2) + Math.pow(y - Yg, 2));

			writeLog("Moving to goal at angle " + angle + " in " + dist
					+ "cm distance");

			turnRobotBalanced(angle, 'r');
			moved = 0;
			while ((moved < dist) && !obstacleFound) {
				moved += stepLength;
				moveRobot(stepLength);
				if (obstacleInFront()) {
					writeLog("Obstacle found at " + getMyPosition());
					obstacleFound = true;
				}
			}

			if (obstacleFound) {
				turnRobotBalanced((int) Math.signum((Math.random() - 0.5))
						* (90 + (int) (Math.random() * 45)), 'r');
				measurement = getDistance();
				moveRobot(Math.min(measurement.get("frontRight") - 5, Math.min(
						measurement.get("frontLeft") - 5,
						Math.min(measurement.get("frontMiddle") - 5, 50))));
			}
			if (Math.sqrt(Math.pow(x - Xg, 2) + Math.pow(y - Yg, 2)) < stepLength + 1) {
				goalReached = true;
			}
		}
		
		turnRobotBalanced(theta - angle,'r');
	}

	/**
	 * Uses move robot by velocity to drive to a given goal destination while circumventing obstacles
	 * by rotating by a random angle and moving a couple of centimeters everytime an obstacle is found
	 * @param x the goal's x-coordinate
	 * @param y the goal's y-coordinate
	 * @param theta once the robot reaches the goal destination it should rotate to be in this angle
	 */
	public void moveToGoalNaive3(double x, double y, int theta) {
		int dist;
		int angle = 0;
		int TOL = 8;
		boolean obstacleFound;
		boolean goalReached = false;

		while (!goalReached) {
			obstacleFound = false;

			angle = getAngleToGoal(x, y);
			dist = getDistanceToGoal(x,y);

			writeLog("Moving to goal at angle " + angle + " in " + dist
					+ "cm distance");

			turnRobotBalanced(angle, 'r');
			if (!moveByVelocity(dist, true)) {
				obstacleFound = true;
				writeLog("Obstacle found at " + getMyPosition());
			}

			if (obstacleFound) {
				turnRobotBalanced((int) Math.signum((Math.random() - 0.5))
						* (90 + (int) (Math.random() * 20)), 'r');
				moveByVelocity(50, true);
			}
			if (Math.sqrt(Math.pow(x - Xg, 2) + Math.pow(y - Yg, 2)) < TOL) {
				goalReached = true;
			}
		}

		turnRobotBalanced(theta - angle, 'r');
	}

	/**
	 * robot tries to rotate parallel to any wall or obstacle, from every angle
	 * he drives to it
	 */
	public void rotateToWall() {
		Map<String, Integer> measurement = getDistance();
		double TOL = 10;
		int start = 1;
		boolean atTheEdge = false;
		if (measurement.get("frontLeft") > 200) {
			atTheEdge = true;
			while (measurement.get("frontRight") < 200 / 2) {
				turnRobot(start++, 'l');
				measurement = getDistance();
			}
			turnRobot(180, 'r');
		}
		start = 1;
		if (measurement.get("frontRight") > 200 / 2) {
			atTheEdge = true;
			while (measurement.get("frontLeft") < 200) {
				turnRobot(start++, 'r');
				measurement = getDistance();
			}
			turnRobot(180, 'l');
		}
		// Robot is now parallel to wall

		start = 1;
		while (measurement.get("frontLeft") / 2 - measurement.get("frontRight") < TOL
				&& atTheEdge == false) {
			turnRobot(start++, 'r');
			measurement = getDistance();
		}
	}

	/**
	 * robot drives completely around obstacle and measures distance to goal
	 * after each step after he made his round, he drives back to nearest
	 * position to goal (on his way around the obstacle) and heads to it
	 * 
	 * @param goalX
	 * @param goalY
	 */
	public void roundObstacle(int goalX, int goalY) {
		Position startPosition = getMyPosition();
		Position closestPosition = startPosition;
		boolean startPositionReached = false;
		boolean closestPositionReached = false;
		final double TOL = 10.0; // Tolerated distance for comparison of current
									// and start position
		int movedTotalDistance = 0; // minimum distance needs to exceed a
									// certain value before current and start
									// position get compared

		int curGoalDist = getDistanceToGoal(goalX,goalY); // distance form current position to
											// goal
		int closestDistance = curGoalDist;

		writeLog("Going around obstacle");

		while (!startPositionReached) {
			// Drive around obstacle and find closest position to goal
			while (turnAndCheckRightSide()) {
				// If there is an obstacle in front, turn right and continue
				// if (obstacleInFront()) {
				// turnRobot(90, 'r');
				// }
				moveByVelocity(3, false);

				// boolean breakCond = driveToIntersectionMLine(3, goalX,
				// goalY);
				// if(breakCond == true)
				// moveToGoal(goalX, goalY);

				movedTotalDistance = movedTotalDistance + 3;
				curGoalDist = getDistanceToGoal(goalX,goalY);
				if (curGoalDist < closestDistance) { // Check whether current
														// point is closer to
														// goal or not
					closestDistance = curGoalDist;
					closestPosition = getMyPosition();
					writeLog("Closest point to goal found at ("
							+ getMyPosition().x + "," + getMyPosition().y
							+ ") - distance to goal: " + closestDistance + "cm");
				}
				if ((startPosition.minus(getMyPosition()) < TOL)
						&& movedTotalDistance >= 10) { // Check if start
														// position
														// is reached again
					startPositionReached = true;
					writeLog("Back at starting position");
					break;
				}
			}
			if (!obstacleInFront()) {
				// driveByVelocity(DistToPassObstacleL);
				turnRobot(10, 'r'); // make sure robot cannot hit obstacle with
									// bar
				moveByVelocity(RobotLength + RobotLength / 2, false);
			}
			turnRobot(90, 'l');
		}

		writeLog("Navigating to the closest point (" + closestPosition.x + ","
				+ closestPosition.y + ")");
		robotSetLeds(0, 127);

		// critical code
		while (!closestPositionReached) {
			// Drive around obstacle and find closest position to goal
			robotSetLeds(127, 0);
			try {
				Thread.sleep(30);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			robotSetLeds(0, 127);
			// while (obstacleLeft()) {
			// // If there is an obstacle in front turn right and continue
			// if (obstacleInFront()) {
			// turnRobot(90, 'r');
			// }
			// moveRobot(5);
			// if (closestPosition.minus(getMyPosition()) < TOL) {
			// closestPositionReached = true;
			// writeLog("Closest Point reached");
			// }
			// }
			// // if (!obstacleInFront()) {
			// // moveRobot(DistToPassObstacleL);
			// // }
			// moveRobot(8);
			// turnRobot(90, 'l');
			// moveRobot(8);
			// moveRobot(2);
		}
		// if (!obstacleInFront()) {
		// moveRobot(DistToPassObstacleL);
		// }
		moveRobot(8);
		turnRobot(90, 'l');
	}

	/**
	 * bug 2 algorithm 1) head toward goal on the m-line 2) if an obstacle is in
	 * the way, follow it until you encounter the m-line again closer to the
	 * goal. 3) Leave the obstacle and continue toward the goal
	 * 
	 * @param x
	 * @param y
	 */
	public void bug2(int x, int y) {
		double dist;
		int angle;
		int TOL = 8;
		Boolean[] hitObstacleOrMline;
		angle = getAngleToGoal(x, y);
		dist = Math.sqrt(Math.pow(x - Xg, 2) + Math.pow(y - Yg, 2));
		writeLog("Moving to goal at angle " + angle + " in " + dist
				+ "cm distance");

		turnRobotBalanced(angle, 'r');

		if (!moveByVelocity(dist, true)) {
			writeLog("Going around obstacle");
			turnRobotBalanced(90, 'r');

			do {
				hitObstacleOrMline = driveToIntersectionMLine(
						DistToPassObstacleL, x, y);
				if (!hitObstacleOrMline[1]) { // Obstacle in front
					turnRobotBalanced(45, 'r');
				}

				if (!obstacleLeft()) {
					hitObstacleOrMline = driveToIntersectionMLine(
							DistToPassObstacleL, x, y);
					turnRobotBalanced(45, 'l');
				}
			} while (!hitObstacleOrMline[0]); // not on Mline
		}

		dist = Math.sqrt(Math.pow(x - Xg, 2) + Math.pow(y - Yg, 2));
		if (dist > TOL) {
			bug1(x, y);
		}
	}


	/**
	 * 
	 * @return Position object based on current position
	 */
	public Position getMyPosition() {
		Position myPos = new Position(Xg, Yg, Tg);
		return myPos;
	}
}
