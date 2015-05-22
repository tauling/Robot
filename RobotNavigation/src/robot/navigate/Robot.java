package robot.navigate;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

import robot.opencv.ImageProcessor;
import robot.shapes.Ball;
import robot.shapes.Beacon;
import robot.shapes.Square;
import jp.ksksue.driver.serial.FTDriver;
import android.os.Handler;
import android.util.Log;
import android.widget.ScrollView;
import android.widget.TextView;

public class Robot {

	// TODO Check if some methods could be changed to private

	private TextView textLog; // Textview which displays the log within app.
	private ScrollView svLog; // Allows to scroll the textLog
	private String TAG; // Tag for log-messages sent to logcat

	private ImageProcessor imageProcessor; // Used for image processing.

	// -> Robot Calibration
	private Integer ObsDetectBorderLR = 25; // Measurements in front of the
											// robot
											// below this value are treated as
											// obstacle (Working range of
											// left/right
											// sensor is 10 to 80cm)
	private Integer ObsDetectBorderM = 20; // Measurements below this value are
											// treated as obstacle (Working
											// range of middle sensor is 20 to
											// 80cm)

	private double CorrFactMoveForwardByDist = (1391.0) / (1000.0); // Should be
	// set,
	// such that
	// moveByDistance(100) moves
	// the
	// the robot for 100cm.

	private double CorrFactMoveForwardByVel = (486.0 / 1000.0);// Should be
	// set,
	// such that
	// moveByVelocity(100) moves
	// the
	// the robot for 100cm.
	private double CorrFactMoveForwardByVelSlow = (893.0 / 1000.0); // Should
	// be
	// such that
	// moveByVelocitySlow(100) moves
	// the
	// the robot for 100cm.
	private double CorrFactAngleByDist = (1332.0 / 1000.0); // Should
	// be
	// set, such
	// that
	// turnByDistance(360)
	// rotates for exactly 360 degrees.

	private double CorrFactAngleByVel = (204.0); // Should
	// be
	// set, such
	// that
	// turnByVelocity(360)
	// rotates for exactly 360 degrees.

	private final int IdSensorLeft = 7; // Call findSensorIDs() to determine the
										// corresponding ID
	private final int IdSensorRight = 8; // Call findSensorIDs() to determine
											// the corresponding ID
	private final int IdSensorMiddle = 9; // Call findSensorIDs() to determine
											// the corresponding ID

	// TODO Sensors also have Factors sometimes; Add them here
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

	static final Map<Integer, Point> BeaconPosition; // maps colorID combo
														// (10*upperColorID +
														// lowerColorID) to the
														// position in real
														// world coordinates
	static final Map<Integer, Integer> BeaconID;// maps colorID combo
												// (10*upperColorID +
												// lowerColorID) to the ID
	static final Map<Integer, Integer> BeaconsAngleOffs; // maps beaconID-combo
															// to angles which
															// have to be added
															// to the
															// calculation for
															// gaining the real
															// world coordinates

	static {
		Map<Integer, Point> tmpPosition = new LinkedHashMap<Integer, Point>();
		tmpPosition.put(01, new Point(-125.0, 125.0));
		tmpPosition.put(10, new Point(0, 125.0));
		tmpPosition.put(02, new Point(125.0, 125.0));
		tmpPosition.put(31, new Point(-125.0, 0));
		tmpPosition.put(13, new Point(125.0, 0));
		tmpPosition.put(30, new Point(-125.0, -125.0));
		tmpPosition.put(03, new Point(0, -125.0));
		tmpPosition.put(20, new Point(125.0, -125.0));
		BeaconPosition = Collections.unmodifiableMap(tmpPosition);

		Map<Integer, Integer> tmpID = new LinkedHashMap<Integer, Integer>();
		tmpID.put(01, 1);
		tmpID.put(10, 2);
		tmpID.put(02, 3);
		tmpID.put(31, 4);
		tmpID.put(13, 5);
		tmpID.put(30, 6);
		tmpID.put(03, 7);
		tmpID.put(20, 8);
		BeaconID = Collections.unmodifiableMap(tmpID);

		Map<Integer, Integer> tmpAngles = new LinkedHashMap<Integer, Integer>();
		tmpAngles.put(12, 90);
		tmpAngles.put(13, 90);
		tmpAngles.put(23, 90);
		tmpAngles.put(35, 180);
		tmpAngles.put(38, 180);
		tmpAngles.put(58, 180);
		tmpAngles.put(67, 270);
		tmpAngles.put(68, 270);
		tmpAngles.put(78, 270);
		tmpAngles.put(14, 0);
		tmpAngles.put(16, 0);
		tmpAngles.put(46, 0);
		tmpAngles.put(24, 45);
		tmpAngles.put(25, 135);
		tmpAngles.put(57, 225);
		tmpAngles.put(47, 315);
		BeaconsAngleOffs = Collections.unmodifiableMap(tmpAngles);
	}

	// <- Robot Calibration

	// TODO Check the use of WriteLog; too many comments in textLog while
	// playing with the robot.

	private Position myPos = new Position(0, 0, 0); // Current position of the
													// robot.

	private int balancedAngle = 0; // Helper variable used to track the
									// difference between total left and right
									// turns.

	private FTDriver com; // Connection to the serial bus.
	private Handler mhandler = new Handler(); // Thread handler to display logs
												// asynchronously

	/**
	 * Constructor method.
	 * 
	 * @param textLog
	 *            Textview used for logging.
	 * @param svLog
	 *            Scrollview to enable scrolling on textLog
	 * @param com
	 *            Connection to the serial bus.
	 * @param TAG
	 *            Used for messages to logcat.
	 * @param imageProcessor
	 *            used for image processing.
	 */
	public Robot(TextView textLog, ScrollView svLog, FTDriver com, String TAG,
			ImageProcessor imageProcessor) {
		this.textLog = textLog;
		this.svLog = svLog;
		this.com = com;
		this.imageProcessor = imageProcessor;
		this.TAG = TAG;
	}

	/**
	 * 
	 * @author Gregor
	 * 
	 *         Helper class which allows asynchronous access to the textview for
	 *         logging purposes.
	 */
	private class WriteLogRunnable implements Runnable {

		private String text = null; // Text to write to log.

		/**
		 * Constructor
		 * 
		 * @param text
		 *            which is written to log
		 */
		public WriteLogRunnable(String text) {
			this.text = text;
		}

		/**
		 * Starts a thread which writes the text to the log file.
		 */
		public void run() {

			mhandler.post(new Runnable() {
				public void run() {
					Log.i(TAG, "Robot log: " + text);
					textLog.append(text);
					svLog.fullScroll(ScrollView.FOCUS_DOWN);
				}
			});
		}
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

	/**
	 * set bar with value between 0 and 260
	 * 
	 * @param value
	 */
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
	 * transfers given bytes via the serial connection.
	 * 
	 * @param data
	 *            bytecode that is sent to the robot
	 */
	private void comWrite(byte[] data) {
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
	private String comRead() {
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
	private String comReadWrite(byte[] data) {
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
	private String comReadWrite(byte[] data, int time) {
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
		}
	}

	/**
	 * Add the given integer to the log file.
	 * 
	 * @param value
	 */
	public void writeLog(int value) {
		writeLog((long) value);
	}

	/**
	 * Add the given long integer to the log file.
	 * 
	 * @param value
	 */
	public void writeLog(long value) {
		new Thread(new WriteLogRunnable(value + "\n")).start();
	}

	// TODO update description
	// TODO needed?
	// TODO If so, update with different correcting factors depending on angle
	public void turnByVelocity(int angle, char dir) {

		writeLog("turnByVelocity " + angle);
		double start = System.currentTimeMillis(); // [ms]
		double curTime = start;
		int velocity = 11;
		double speed = velocity / CorrFactAngleByVel;
		double corrTime = reduceAngle(angle) / speed; // [ms]
		double end = start + corrTime;
		robotSetLeds(0, 127);

		switch (dir) {
		case 'l':
			velocity = -velocity;
			break;
		case 'r':
			break;
		}

		comReadWrite(new byte[] { 'i', (byte) velocity, (byte) -velocity, '\r',
				'\n' });
		while (curTime <= end) {
			curTime = System.currentTimeMillis();
		}
		comReadWrite(new byte[] { 'i', (byte) 0, (byte) 0, '\r', '\n' });
		double movedTime = curTime - start;

		updateRotation((int) (movedTime * speed), 'r');
	}

	/**
	 * Drive by velocity (fast), update position afterwards.
	 * 
	 * @param dist
	 *            distance to drive [cm]
	 * @param stopOnObstacle
	 *            when true, stop before hitting obstacle
	 * @return true when there is no obstacle in front, false otherwise
	 */
	public Boolean moveByVelocity(double dist, boolean stopOnObstacle) {
		return moveByVelocity(dist, stopOnObstacle, 15,
				CorrFactMoveForwardByVel);
	}

	/**
	 * Drive by velocity (slow), update position afterwards.
	 * 
	 * @param dist
	 *            distance to drive [cm]
	 * @param stopOnObstacle
	 *            when true, stop before hitting obstacle
	 * @return true when there is no obstacle in front, false otherwise
	 */
	public Boolean moveByVelocitySlow(double dist, boolean stopOnObstacle) {
		return moveByVelocity(dist, stopOnObstacle, 11,
				CorrFactMoveForwardByVelSlow);
	}

	/**
	 * Drive by velocity, update position afterwards.
	 * 
	 * @param dist
	 *            distance to drive [cm]
	 * @param stopOnObstacle
	 *            when true, stop before hitting obstacle
	 * @param velocity
	 *            velocity of the robot
	 * @param corrFactMoveVel
	 *            depending on the velocity, different correcting factors are
	 *            needed (calibrate!)
	 * @return true when there is no obstacle in front, false otherwise
	 */
	private Boolean moveByVelocity(double dist, boolean stopOnObstacle,
			int velocity, double corrFactMoveVel) {
		double start = System.currentTimeMillis(); // [ms]
		double curTime = start;
		double speed = (100.0 / (1000 * velocity)) / corrFactMoveVel; // [cm/ms]
		double corrTime = Math.abs(dist / speed); // [ms]
		double end = start + corrTime;
		boolean freeWay = true;
		robotSetLeds(0, 127);
		comReadWrite(new byte[] { 'i',
				(byte) (velocity * ((int) Math.signum(dist))),
				(byte) (velocity * ((int) Math.signum(dist))), '\r', '\n' });
		while (curTime <= end && (freeWay || !stopOnObstacle)) {
			if (obstacleInFront()) {
				freeWay = false;
				robotSetLeds(127, 0);
			}
			curTime = System.currentTimeMillis();
		}
		comReadWrite(new byte[] { 'i', (byte) 0, (byte) 0, '\r', '\n' });
		double movedTime = curTime - start;

		updatePosition((int) (Math.signum(dist) * movedTime * speed));
		return freeWay;
	}

	/**
	 * updates global Position parameters after Robot moved one stepLength
	 * 
	 * @param stepLength
	 */
	private void updatePosition(int stepLength) {
		double movementX = 0, movementY = 0;
		movementX = Math.sin(Math.toRadians((double) myPos.theta)) * stepLength;
		movementY = Math.cos(Math.toRadians((double) myPos.theta)) * stepLength;
		myPos.x += movementX;
		myPos.y += movementY;
		writeLog("my Position: (" + myPos.x + "," + myPos.y + "," + myPos.theta
				+ ")");
	}

	/**
	 * Calculates the angle to the target (global coordinates).
	 * 
	 * @param x
	 *            x-coordinate of the target
	 * @param y
	 *            y-coordinate of the target
	 * @return angle to the target (global coordinates)
	 */
	private int getAngleToTarget(double x, double y) {
		int angle = (int) (Math.toDegrees(Math.atan2(x - myPos.x, y - myPos.y)));

		return reduceAngle(angle - myPos.theta);
	}

	/**
	 * Calculates the distance between the robot and the target.
	 * 
	 * @param x
	 *            x-coordinate of the target
	 * @param y
	 *            y-coordinate of the target
	 * @return distance between robot and target in cm
	 */
	private int getDistanceToTarget(double x, double y) {
		return (int) Math.sqrt(Math.pow(x - myPos.x, 2)
				+ Math.pow(y - myPos.y, 2));
	}

	/**
	 * Raises and lowers the bar.
	 * 
	 * @param d
	 *            "+" for rising bar; "-" for lowering bar.
	 */
	public void moveBar(char d) {

		switch (d) {
		case '+':
			writeLog("Rising bar");
			comReadWrite(new byte[] { '+', '\r', '\n' });
			break;
		case '-':
			writeLog("Lowering bar");
			comReadWrite(new byte[] { '-', '\r', '\n' });
			break;
		}

	}

	/**
	 * Ensures that angle is always a value between 0 and 360°.
	 * 
	 * @param angle
	 *            angle to reduce
	 * @return angle between 0 and 360°
	 */
	private int reduceAngle(int angle) {
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
	private void updateRotation(int angle, char dir) {
		switch (dir) {
		case 'l':
			myPos.theta -= angle;
			break;
		case 'r':
			myPos.theta += angle;
			break;
		default:
			System.out.println("Wrong input direction");
			break;
		}
		myPos.theta = reduceAngle(myPos.theta);
		writeLog("my Position: (" + myPos.x + "," + myPos.y + "," + myPos.theta
				+ ")");
	}

	/**
	 * move robot by a specific distance
	 * 
	 * @param dist
	 *            distance in centimeters
	 */
	public void moveByDistance(int dist) {
		int corrDist = (int) (dist * CorrFactMoveForwardByDist);
		int waitTimeFact = 100;
		while (Math.abs(corrDist) > 127) { // Byte stores values from -128 to
											// 127
			corrDist -= (int) (Math.signum(corrDist)) * 127;
			comReadWrite(new byte[] { 'k',
					(byte) ((int) (Math.signum(corrDist)) * 127), '\r', '\n' },
					(int) (127 / CorrFactMoveForwardByDist) * waitTimeFact);
		}
		comReadWrite(new byte[] { 'k', (byte) corrDist, '\r', '\n' },
				(int) Math.abs(corrDist / CorrFactMoveForwardByDist)
						* waitTimeFact);
		writeLog("moving by distance");
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
		double x_ofIntersection = (myPos.y - (myPos.x * Math.tan(Math
				.toRadians(myPos.theta))))
				/ ((((double) goal_y) / goal_x) - (Math.tan(Math
						.toRadians(myPos.theta))));
		double y_ofIntersection = x_ofIntersection * ((double) goal_y) / goal_x;
		double dist = Math.min(
				getDistanceToTarget(x_ofIntersection, y_ofIntersection),
				max_dist);

		Boolean[] ret = new Boolean[2];

		ret[1] = moveByVelocity(dist, true);

		if (ret[1] && (dist < max_dist)) {
			ret[0] = true;
		} else {
			ret[0] = false;
		}

		return ret;
	}

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
	 * @return false if it found an obstacle, true if way was free (if
	 *         obstacletreatment = 1)
	 */
	public Boolean moveSquare(int dist, char dir, int obstacleTreatment) {
		Boolean ret = true;
		switch (obstacleTreatment) {
		case 0:
			for (int i = 0; i < 4; i++) {
				turnByDistanceBalanced(90, dir);
				moveByVelocity(dist, false);
			}
			turnByDistanceBalanced(getAngleToTarget(0, 0), 'r');
			moveByVelocity(getDistanceToTarget(0, 0), false);
			turnByDistanceBalanced(myPos.theta, 'l');
			robotSetLeds(127, 127);
			break;
		case 1:
			turnByDistanceBalanced(90, dir);
			Boolean freeway = true;
			for (int i = 0; i < 4; i++) {
				if (freeway && moveByVelocity(dist, true)) {
					turnByDistanceBalanced(90, dir);
				} else {
					freeway = false;
					ret = false;
				}
			}
			if (freeway) {
				turnByDistanceBalanced(getAngleToTarget(0, 0), 'r');
				moveByVelocity(getDistanceToTarget(0, 0), false);
				turnByDistanceBalanced(myPos.theta, 'l');
			}
			robotSetLeds(127, 127);
			break;
		case 2:
			switch (dir) {
			case 'r':
				moveToGoalNaive3(dist, 0, 90);
				moveToGoalNaive3(dist, dist, 0);
				moveToGoalNaive3(0, dist, 270);
				moveToGoalNaive3(0, 0, 180);
				break;
			case 'l':
				moveToGoalNaive3(-dist, 0, 270);
				moveToGoalNaive3(-dist, dist, 0);
				moveToGoalNaive3(0, dist, 90);
				moveToGoalNaive3(0, 0, 0);
				break;
			}
			robotSetLeds(127, 127);
		}

		return ret;
	}

	/**
	 * Tells the robot to turn
	 * 
	 * @param angle
	 *            in degrees
	 * @param dir
	 *            ("l" = left; "r" = right)
	 */
	public void turnByDistanceBalanced(int angle, char dir) {
		angle = reduceAngle(angle);
		switch (dir) {
		case 'l':
			if (Math.abs(balancedAngle - angle) < Math.abs(balancedAngle
					+ (360 - angle))) {
				balancedAngle -= angle;
				turnByDistance(angle, 'l');
			} else {
				balancedAngle += (360 - angle);
				turnByDistance(360 - angle, 'r');
			}
			break;
		case 'r':
			if (Math.abs(balancedAngle + angle) < Math.abs(balancedAngle
					- (360 - angle))) {
				balancedAngle += angle;
				turnByDistance(angle, 'r');
			} else {
				balancedAngle -= (360 - angle);
				turnByDistance(360 - angle, 'l');
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
	public void turnByDistance(int angle, char dir) {
		int waitTimeFact = 17;
		angle = reduceAngle(angle);
		updateRotation(angle, dir);
		int degrees = angle;
		degrees = (int) (CorrFactAngleByDist * degrees);
		// int targetedAngleByOnce = 30; // TODO: Make global? Check if needed;
		// could improve accuracy 
		int maxDegreesByOnce = 40;
		// if (angle > targetedAngleByOnce) {
		// maxDegreesByOnce = Math.min(127,
		// Math.abs(degrees/(angle/targetedAngleByOnce)));
		// } else {
		// maxDegreesByOnce = 127;
		// }
		switch (dir) {
		case 'r':
			degrees = -degrees;
			break;
		}
		while (Math.abs(degrees) > maxDegreesByOnce) { // Byte stores values
														// from -128 to
			// 127
			degrees -= (int) (Math.signum(degrees)) * maxDegreesByOnce;
			writeLog(comReadWrite(
					new byte[] {
							'l',
							(byte) ((int) (Math.signum(degrees)) * maxDegreesByOnce),
							'\r', '\n' }, waitTimeFact * maxDegreesByOnce));
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
				turnByDistance(90, 'l');
				try {
					Thread.sleep(1000);
				} catch (Exception e) {
				}
				writeLog(comReadWrite(new byte[] { 'u', (byte) 0, (byte) 127,
						'\r', '\n' }));
				comReadWrite(new byte[] { 'i', 15, 15, '\r', '\n' });
				i++;
			}
		}
		comReadWrite(new byte[] { 'i', 0, 0, '\r', '\n' });
	}

	/**
	 * methods tests whether an obstacle is within the range of the 3 front
	 * sensors or not
	 * 
	 * @return true - obstacle detected; false - free way
	 */
	private Boolean obstacleInFront() {
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
	 * Uses move robot by distance to drive to a given goal destination while
	 * circumventing obstacles by rotating by a random angle and moving a couple
	 * of centimeters everytime an obstacle is found
	 * 
	 * @param x
	 *            the goal's x-coordinate
	 * @param y
	 *            the goal's y-coordinate
	 * @param theta
	 *            once the robot reaches the goal destination it should rotate
	 *            to be in this angle
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

			angle = getAngleToTarget(x, y);
			dist = (int) Math.sqrt(Math.pow(x - myPos.x, 2)
					+ Math.pow(y - myPos.y, 2));

			writeLog("Moving to goal at angle " + angle + " in " + dist
					+ "cm distance");

			turnByDistanceBalanced(angle, 'r');
			robotSetLeds(127, 0);
			moved = 0;
			while ((moved < dist) && !obstacleFound) {
				moved += stepLength;
				moveByDistance(stepLength);
				if (obstacleInFront()) {
					writeLog("Obstacle found at " + myPos);
					obstacleFound = true;
				}
			}

			if (obstacleFound) {
				robotSetLeds(0, 127);
				turnByDistanceBalanced((int) Math.signum((Math.random() - 0.5))
						* (90 + (int) (Math.random() * 45)), 'r');
				measurement = getDistance();
				moveByDistance(Math.min(measurement.get("frontRight") - 5, Math
						.min(measurement.get("frontLeft") - 5, Math.min(
								measurement.get("frontMiddle") - 5, 50))));
			}
			if (Math.sqrt(Math.pow(x - myPos.x, 2) + Math.pow(y - myPos.y, 2)) < stepLength + 1) {
				goalReached = true;
			}
		}

		turnByDistanceBalanced(theta - myPos.theta, 'r');
		robotSetLeds(127, 127);
	}

	/**
	 * Uses move robot by velocity to drive to a given goal destination while
	 * circumventing obstacles by rotating by a random angle and moving a couple
	 * of centimeters everytime an obstacle is found
	 * 
	 * @param x
	 *            the goal's x-coordinate
	 * @param y
	 *            the goal's y-coordinate
	 * @param theta
	 *            once the robot reaches the goal destination it should rotate
	 *            to be in this angle
	 */
	public void moveToGoalNaive3(double x, double y, int theta) {
		int dist;
		int angle = 0;
		int TOL = 8;
		boolean obstacleFound;
		boolean goalReached = false;

		while (!goalReached) {
			obstacleFound = false;

			angle = getAngleToTarget(x, y);
			dist = getDistanceToTarget(x, y);

			writeLog("Moving to goal at angle " + angle + " in " + dist
					+ "cm distance");

			turnByDistanceBalanced(angle, 'r');
			robotSetLeds(0, 127);
			if (!moveByVelocity(dist, true)) {
				obstacleFound = true;
				robotSetLeds(127, 0);
				writeLog("Obstacle found at " + myPos);
			}

			if (obstacleFound) {
				turnByDistanceBalanced((int) Math.signum((Math.random() - 0.5))
						* (90 + (int) (Math.random() * 20)), 'r');
				moveByVelocity(50, true);
			}
			if (getDistanceToTarget(x, y) < TOL) {
				goalReached = true;
			}
		}

		turnByDistanceBalanced(theta - myPos.theta, 'r');
		robotSetLeds(127, 127);
	}

	/**
	 * Approaches target point (x, y) and stops offset centimeters before the
	 * target. Does not change angle once it stops. Does not check whether or
	 * not the robot stops within the tolerance (otherwise it might kick out a
	 * ball near target point)!
	 * 
	 * @param x
	 *            x-coordinate of the target [cm]
	 * @param y
	 *            y-coordinate of the target [cm]
	 * @param offset
	 *            offset to the target [cm]
	 */
	public void moveToTargetWithoutAngle(double x, double y, double offset) {
		int totalAngleToGoal = getAngleToTarget(x, y) + getMyPosition().theta;
		moveToTarget(x, y, totalAngleToGoal, offset, false);
	}

	/**
	 * Drives to the target point (x, y). Does not change angle once it stops.
	 * 
	 * @param x
	 *            x-coordinate of the target [cm]
	 * @param y
	 *            y-coordinate of the target [cm]
	 */
	public void moveToTarget(double x, double y) {
		int totalAngleToGoal = getAngleToTarget(x, y) + getMyPosition().theta;
		moveToTarget(x, y, totalAngleToGoal);
	}

	/**
	 * Drives to the target point (x, y, theta).
	 * 
	 * @param x
	 *            x-coordinate of the target [cm]
	 * @param y
	 *            y-coordinate of the target [cm]
	 * @param theta
	 *            angle at target point (global coordinates)
	 */
	public void moveToTarget(double x, double y, double theta) {
		moveToTarget(x, y, theta, 0.0);
	}

	/**
	 * Drives to the target point (x, y, theta).
	 * 
	 * @param target
	 *            (x, y, theta) of the target point.
	 */
	public void moveToTarget(Position target) {
		moveToTarget(target.x, target.y, target.theta, 0.0);
	}

	/**
	 * Approaches target point (x, y, theta) and stops offset centimeters before
	 * the target.
	 * 
	 * @param x
	 *            x-coordinate of the target [cm]
	 * @param y
	 *            y-coordinate of the target [cm]
	 * @param theta
	 *            angle at target point (global coordinates)
	 * @param offset
	 *            offset to the target [cm]
	 */
	public void moveToTarget(double x, double y, double theta, double offset) {
		moveToTarget(x, y, theta, offset, true);
	}

	/**
	 * Approaches target point (x, y, theta) and stops offset centimeters before
	 * the target.
	 * 
	 * @param x
	 *            x-coordinate of the target [cm]
	 * @param y
	 *            y-coordinate of the target [cm]
	 * @param theta
	 *            angle at target point (global coordinates)
	 * @param offset
	 *            offset to the target [cm]
	 * @param checkTOL
	 *            when enabled, it is ensured that the robot ends up at the
	 *            target within the given tolerance
	 */
	private void moveToTarget(double x, double y, double theta, double offset,
			boolean checkTol) {
		int angle = 0, dist;
		Boolean goalReached = false;
		int TOL = 5;
		do {
			angle = getAngleToTarget(x, y);
			dist = getDistanceToTarget(x, y);

			writeLog("Moving to goal at angle " + angle + " in " + dist
					+ "cm distance" + " offset of " + offset);
			dist = (int) (dist - offset);

			turnByDistanceBalanced(angle, 'r');
			robotSetLeds(127, 0);
			moveByVelocity(dist, false);
			if (Math.abs((getDistanceToTarget(x, y) - offset)) < TOL) {
				goalReached = true;
			}
		} while (!goalReached && checkTol);
		turnByDistanceBalanced((int) theta - myPos.theta, 'r');
		robotSetLeds(127, 127);
	}

	/**
	 * Resets the robot's current position to (0, 0, 0)
	 */
	public void resetPosition() {
		myPos = new Position(0.0, 0.0, 0);
	}

	/**
	 * Returns the robot's current position.
	 * 
	 * @return current robot's position.
	 */
	public Position getMyPosition() {
		return myPos;
	}

	// TODO needed? (gets called inside detectBalls)
	// TODO if so, update comment
	/**
	 * robot aligns his body to a surrendered ball
	 * 
	 * @param ball
	 * @param imageWidth
	 *            width of the camera frame's image
	 * @return TRUE, after he turned enough
	 */
	public Boolean alignToBall(Ball ball, double imageWidth) {
		Boolean aligned = false;
		double centerXAxis = imageWidth / 2;
		double TOL = 2.0;
		while (!aligned) {
			Point ballCenter = ball.getBallCenterCameraFrame();
			double ballXAxis = ballCenter.x;
			double diff = centerXAxis - ballXAxis;
			if (Math.abs(diff) > TOL && diff < 0) {
				turnByDistanceBalanced(15, 'r');
			} else if (Math.abs(diff) > TOL && diff < 0) {
				turnByDistanceBalanced(10, 'l');
			}

		}
		return aligned;
	}

	/**
	 * Detects a ball, cages it, and moves it to the given target point.
	 * 
	 * @param targetX
	 *            x-coordinate of the target point
	 * @param targetY
	 *            y-coordinate of the target point
	 * @param mRgbaWork
	 *            image to work on
	 * @param myColors
	 *            colors which are recognized by the robot.
	 * @param homographyMatrix
	 *            used to map camera pixels to ground plane coordinates
	 */
	public void findAndDeliverBall(double targetX, double targetY,
			Mat mRgbaWork, List<Scalar> myColors, Mat homographyMatrix) {
		Ball myBall = detectOneBall(mRgbaWork, myColors, homographyMatrix);
		if (myBall != null) {
			driveToBallAndCage(myBall, mRgbaWork, myColors, homographyMatrix);
			moveToTargetWithoutAngle(targetX, targetY, 5);
			moveByVelocitySlow(-16, false);
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
			}
			robotSetBar(126);
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
			}
			moveByVelocity(-35, false);
		}
	}

	/**
	 * Turns robot for a maximum of 360°, stops when ball is adjusted to the
	 * center of the camera frame.
	 * 
	 * @param mRgbaWork
	 *            image to work on
	 * @param myColors
	 *            list of colors that are recognized by the robot
	 * 
	 * @return true if ball is found, false otherwise.
	 */
	public boolean turnAndFindABall(Mat mRgbaWork, List<Scalar> myColors) {

		Boolean foundBall = false;
		int turnedAngle = 0;
		List<Point> circles = new ArrayList<Point>();
		while (turnedAngle < 360 && !foundBall) {
			circles = imageProcessor.findCirclesOnCamera(mRgbaWork, myColors);
			if (circles.size() > 0) {
				Log.i(TAG, "found circle x:" + circles.get(0).x + " y:"
						+ circles.get(0).y);
				alignToPoint(circles.get(0), mRgbaWork, myColors);
				Log.i(TAG, "(turnAndFindABall) found a ball");
				foundBall = true;
			} else {
				turnByDistanceBalanced(25, 'r');
				turnedAngle += 25;
			}
		}
		Log.i(TAG, "(turnAndFindABall) Finished");
		return foundBall;

	}

	/**
	 * Drives to the ball and cages it.
	 * 
	 * @param ball
	 *            the ball to cage
	 * @param mRgbaWork
	 *            image to work on
	 * @param myColors
	 *            colors that are recognized by the robot
	 * @param homographyMatrix
	 *            used to map from camera pixels to ground plane coordinates
	 */
	public void driveToBallAndCage(Ball ball, Mat mRgbaWork,
			List<Scalar> myColors, Mat homographyMatrix) {
		Point ballTarget = ball.getPosGroundPlane();
		moveToTargetWithoutAngle(ballTarget.x, ballTarget.y, 25);
		ballTarget = detectOneBall(mRgbaWork, myColors, homographyMatrix)
				.getPosGroundPlane();
		moveToTargetWithoutAngle(ballTarget.x, ballTarget.y, 5);
		Log.i(TAG, "(driveToBallAndCage) lowering bar");
		robotSetBar(50);
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
		}
	}

	// TODO: needed?
	// TODO add comment
	// TODO fix: this method is used to detect various balls; currently not
	// working
	// TODO: Should probably be split up between imageProcessor and Robot class
	// TODO: Afterwards update visibility of methods in imageProcessor again
	public void detectBalls(Mat img, List<Ball> foundBalls, Mat mRgbaOutput) {
		List<MatOfPoint> contours = imageProcessor.findContours(img);
		for (MatOfPoint area : contours) {

			Point center = imageProcessor.computeCenterPt(area);
			// Point pointGroundPlane = computePointGroundPlane();
			Point pointGroundPlane = null; // TODO implement (don't forget to
											// add the robot's pos coordinates)
			double rad = imageProcessor.computeRadius(area, center);

			Ball detectedBall = new Ball(center, pointGroundPlane, rad);

			alignToBall(detectedBall, mRgbaOutput.width());

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
	 * Detects a ball with given colors in given camera frame.
	 * 
	 * @param mRgbaWork
	 *            input image
	 * @param myColors
	 *            colors to look for
	 * @param homographyMatrix
	 *            used to map camera pixels to ground plane coordinates
	 * 
	 * @return Ball object if found, null otherwise.
	 */
	public Ball detectOneBall(Mat mRgbaWork, List<Scalar> myColors,
			Mat homographyMatrix) {
		Ball detectedBall = null;
		if (turnAndFindABall(mRgbaWork, myColors)) {
			for (Scalar hsvColor : myColors) {
				Mat grayImg;
				do {
					grayImg = imageProcessor.filter(mRgbaWork, hsvColor);
				} while (grayImg.empty());
				List<MatOfPoint> contours = imageProcessor
						.findContours(grayImg);
				Log.e(TAG, "found areas: " + contours.size());
				for (MatOfPoint area : contours) {

					Point center = imageProcessor.computeCenterPt(area);
					double rad = imageProcessor.computeRadius(area, center);
					Point lowestPoint = new Point(center.x, center.y + rad);
					Point pointGroundPlane = getGroundPlaneCoordinates(
							lowestPoint, homographyMatrix);

					detectedBall = new Ball(center, pointGroundPlane, rad);
				}
				grayImg.release();
			}
		}

		return detectedBall;
	}

	/**
	 * Takes a camera point and calculates its ground plane coordinates.
	 * 
	 * @param cameraPoint
	 *            the pixel to map to the ground plane
	 * @param homographyMatrix
	 *            the matrix that maps camera pixels to the ground plane
	 * @return ground plane coordinates of camera point
	 */
	public Point getGroundPlaneCoordinates(Point cameraPoint,
			Mat homographyMatrix) {
		double[] polarCoord = getGroundPlaneCoordinatesRelRobot(cameraPoint,
				homographyMatrix);
		double theta = polarCoord[1];
		double dist = polarCoord[0];
		double dx = -dist
				* Math.sin(2 * Math.PI
						- (theta + Math.toRadians(getMyPosition().theta)));
		double dy = dist
				* Math.cos(2 * Math.PI
						- (theta + Math.toRadians(getMyPosition().theta)));

		Point pointGroundCoord = new Point();
		pointGroundCoord.x = getMyPosition().x + dx;
		pointGroundCoord.y = getMyPosition().y + dy;

		return pointGroundCoord;
	}

	/**
	 * Takes a camera point and calculates its ground plane coordinates relative
	 * to the robot.
	 * 
	 * @param cameraPoint
	 *            the pixel to map to the ground plane
	 * @param homographyMatrix
	 *            the matrix that maps camera pixels to the ground plane
	 * @return ground plane coordinates of camera point relative to robot
	 */
	public double[] getGroundPlaneCoordinatesRelRobot(Point cameraPoint,
			Mat homographyMatrix) {

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
		double theta = Math.atan2(pointGroundCoord.x, pointGroundCoord.y);
		double dist = Math.sqrt(Math.pow(pointGroundCoord.x, 2)
				+ Math.pow(pointGroundCoord.y, 2));

		double[] polarCoord = new double[2];

		polarCoord[0] = dist;
		polarCoord[1] = theta;

		src.release();
		dest.release();
		return polarCoord;
	}

	// TODO: Fix: Is just able to work with one blob on the image.
	// TODO: In case of no fix: Update this description.
	/**
	 * robot aligns his body to a surrendered Point
	 * 
	 * @param point
	 *            camera pixel to align to (horizontally)
	 * @param mRgbaWork
	 *            image which is used to adjust alignment.
	 * @param Colors
	 *            which are recognized by the robot.
	 */
	public void alignToPoint(Point p, Mat mRgbaWork, List<Scalar> myColors) {
		writeLog("(alignToPoint) p = " + p.toString());
		Boolean aligned = false;
		double centerXAxis = mRgbaWork.width() / 2;
		writeLog("(alignToPoint) robot Camera xAxis: " + centerXAxis);
		double TOL = 150.0;
		double ballXAxis = p.x;
		while (!aligned) {
			ballXAxis = imageProcessor.findCirclesOnCamera(mRgbaWork, myColors)
					.get(0).x;
			double diff = centerXAxis - ballXAxis;
			if (Math.abs(diff) > TOL && diff < 0) {
				turnByDistanceBalanced(30, 'r');
			} else if (Math.abs(diff) > TOL && diff > 0) {
				turnByDistanceBalanced(25, 'l');
			} else {
				aligned = true;
			}
		}
		writeLog("(alignToPoint) aligned");
	}

	// TODO: implement
	/**
	 * Uses beacons (with known positions) to update current position.
	 * 
	 * @param beacons
	 *            list of currently seen beacons
	 * @param homographyMatrix
	 *            matrix which was received with the checkboard pattern
	 */
	public void updateGlobalPosition(List<Square> beacons, Mat homographyMatrix) {

	}

	// TODO: implement; see sheet of paper that Gregor wrote
	/**
	 * Takes two visible beacons as input, calculates the global position using
	 * these two beacons.
	 * 
	 * @param beacon1
	 *            first visible beacon
	 * @param beacon2
	 *            second visible beacon
	 * @param homographyMatrix
	 *            matrix which was received with the checkboard pattern
	 * @return current globalPosition
	 */
	private Position findPosition(Beacon beacon1, Beacon beacon2,
			Mat homographyMatrix) {

		// Creates the ID of a pair of beacons which consists of two numbers:
		// First number -> ID of beacon with lower ID,
		// Second number -> ID of beacon with higher ID
		int beacIDcomb;
		if (BeaconID.get(beacon1.getColorComb()) > BeaconID.get(beacon2
				.getColorComb())) {
			beacIDcomb = BeaconID.get(beacon1.getColorComb()) * 10
					+ BeaconID.get(beacon2.getColorComb());
		} else {
			beacIDcomb = BeaconID.get(beacon2.getColorComb()) * 10
					+ BeaconID.get(beacon1.getColorComb());
		}

		// distance between the two beacons
		double b = imageProcessor.distPointToPoint(
				BeaconPosition.get(beacon1.getColorComb()),
				BeaconPosition.get(beacon2.getColorComb()));

		double c;
		double a;
		double[] ground1 = getGroundPlaneCoordinatesRelRobot(
				beacon1.getLowPt(), homographyMatrix); // TODO: We need
														// coordinates relative
														// to
														// robot here; the used
														// method does
														// something else
		double[] ground2 = getGroundPlaneCoordinatesRelRobot(
				beacon2.getLowPt(), homographyMatrix); // TODO: We need
														// coordinates relative
														// to
														// robot here; the used
														// method does
														// something else

		// The distance of the beacon which appears first (from left to right)
		// on the camera frame is stored in variable c
		// The distance of the other beacon in a
		double thetaRel;
		Point beaconPos;
		if (ground1[1] > ground2[1]) {
			c = ground2[0];
			a = ground1[0];
			thetaRel = ground2[1];
			beaconPos = BeaconPosition.get(beacon2.getColorComb());
		} else {
			c = ground1[0];
			a = ground2[0];
			thetaRel = ground1[1];
			beaconPos = BeaconPosition.get(beacon1.getColorComb());
		}

		// Law of cosine to calculate the angle between the first beacon and the
		// robot (in relation to the line crossing
		// both beacons.
		double alpha = Math.toDegrees(Math.acos(-(Math.pow(a, 2)
				- Math.pow(b, 2) - Math.pow(c, 2))
				/ (2 * b * c)));

		// Angle in global coordinate system between robot and beacon
		double tmptheta = alpha + BeaconsAngleOffs.get(beacIDcomb);

		int theta = reduceAngle((int) (tmptheta - thetaRel));

		// Using alpha and distance to the left beacon aswell as the Position of
		// the beacon, calculate the position of the robot.

		double dx = c * Math.sin(theta);

		double dy = c * Math.cos(theta);

		Point pointGroundCoord = new Point();
		pointGroundCoord.x = beaconPos.x + dx;
		pointGroundCoord.y = beaconPos.y + dy;

		return new Position(pointGroundCoord.x, pointGroundCoord.y, theta);
	}

	// TODO implement first increment: collect one ball which is lying on the
	// robot's way
	// TODO implement second increment: collect more balls which are lying on
	// the robot's way
	/**
	 * Moves to a given target point. Collects balls that are lying on the way
	 * to the target.
	 * 
	 * firstly turn robot to target
	 * 
	 * method uses list of balls and tries to determine which one is the nearest
	 * to catch on the way to the goal, that's the one to cage first...
	 * 
	 * @param targetPoint
	 */
	public void moveToTargetCollBalls(Position targetPoint, List<Ball> balls,
			Mat mRgbaWork, List<Scalar> myColors, Mat homographyMatrix) {
		turnByDistanceBalanced(getAngleToTarget(targetPoint.x, targetPoint.y),
				'r');

		// TODO: write similar method, which doesn't overwrite surrendered ball
		driveToBallAndCage(findNearestBall(balls), mRgbaWork, myColors,
				homographyMatrix);
		moveToTarget(targetPoint);
	}

	// TODO implement
	// moved from robot because we need the updated ball-list after the robot
	// rotated
	/**
	 * Turns and looks for a ball with a clear line of sight.
	 * 
	 * robot should turn until he sees at least one ball, in case he founds
	 * occasionally more than one he chooses the nearest to return
	 * 
	 * @return found ball; null when no ball is found
	 */
	public Ball findNearestBall(List<Ball> foundBalls) {
		while (foundBalls.size() < 1) {
			turnByDistanceBalanced(15, 'r');
			// here is maybe a delay necessary
		}
		Map<Double, Ball> ballGaps = new HashMap<Double, Ball>();
		for (Ball b : foundBalls) {
			Point groundPoint = b.getPosGroundPlane();
			Double distToBall = Math.sqrt(Math.pow(groundPoint.x - myPos.x, 2)
					+ Math.pow(groundPoint.y - myPos.y, 2));
			ballGaps.put(distToBall, b);
		}
		Entry<Double, Ball> nearestBall = null;
		for (Entry<Double, Ball> entry : ballGaps.entrySet()) {
			if (nearestBall == null || nearestBall.getKey() > entry.getKey()) {
				nearestBall = entry;
			}
		}
		return nearestBall.getValue();
	}
}