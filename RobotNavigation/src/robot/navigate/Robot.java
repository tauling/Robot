package robot.navigate;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.opencv.core.Point;

import jp.ksksue.driver.serial.FTDriver;
import android.os.Handler;
import android.widget.ScrollView;
import android.widget.TextView;

public class Robot {

	private TextView textLog;
	private ScrollView svLog;

	public Robot(TextView textLog, ScrollView svLog, FTDriver com) {
		this.Xg = 0;
		this.Yg = 0;
		this.Tg = 0;
		this.textLog = textLog;
		this.svLog = svLog;
		this.com = com;
	}

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
	private Integer ObsDetectBorderL = 25; // Measurements to the left of the
											// robot
	// below this value are treated as
	// obstacle (Working range of left
	// sensor is 10 to 80cm)
	private Integer ObsDetectBorderR = 25; // Measurements to the right of the
											// robot
	// below this value are treated as
	// obstacle (Working range of right
	// sensor is 10 to 80cm)
	private double CorrFactMoveForwardByDist = (5875.0 / 4309.0)
			* (100.0 / 98.0); // Should be
	// set,
	// such that
	// MoveRobot(100) moves
	// the
	// the robot for 100cm.

	private double CorrFactMoveForwardByVel = (303.0 / 650.0) * (100.0 / 98.0)
			* (101.0 / 98.0) * (103.0 / 102.0);
	private double CorrFactAngleByDist = (8.0 / 7.0) * (360.0 / 365.0); // Should be
																	// set, such
																	// that
	private double CorrFactAngleByVel = 1;
	
	// turnRobot(360)
	// rotates for exactly 360 degrees.
	private final int IdSensorLeft = 7; // Call findSensorIDs() to determine the
										// corresponding ID
	private final int IdSensorRight = 8; // Call findSensorIDs() to determine
											// the corresponding ID
	private final int IdSensorMiddle = 9; // Call findSensorIDs() to determine
											// the corresponding ID

	// TODO: Sensors also have Factors sometimes; Add them here
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

	// <- Robot Calibration

	private double Xg = 0, Yg = 0; // Position of the robot.
	private int Tg = 0; // Angle of the robot.
	private int balancedAngle = 0;

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
	
	public int getTg() {
		return Tg;
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

	// TODO Add description
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
	private void writeLog(long value) {
		new Thread(new WriteLogRunnable(value + "\n")).start();
	}

	/**
	 * Resets position of the robot to (0,0,0)
	 */
	public void resetPosition() {
		Xg = 0;
		Yg = 0;
		Tg = 0;
	}
	
	// TODO write description; Fix
	public void turnByVelocity(int angle,char dir) {

		double start = System.currentTimeMillis(); // [ms]
		double curTime = start;
		int velocity = 15;
		writeLog("startTime: " + (int) start);
		double speed = velocity * CorrFactAngleByVel;
		double corrTime = angle / speed; // [ms]
		double end = start + corrTime;
		robotSetLeds(0, 127);

		switch (dir) {
		case 'r':
			velocity = -velocity;
			writeLog("Turning right");
			break;
		case 'l':
			writeLog("Turning left");
		}
		
		comReadWrite(new byte[] { 'i', (byte) velocity, (byte) -velocity, '\r',
				'\n' });
		while (curTime <= end) {
			curTime = System.currentTimeMillis();
		}
		comReadWrite(new byte[] { 'i', (byte) 0, (byte) 0, '\r', '\n' });
		double movedTime = curTime - start;

		updateRotation((int) (movedTime * angle), 'r');		
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
		robotSetLeds(0, 127);
		comReadWrite(new byte[] { 'i', (byte) velocity, (byte) velocity, '\r',
				'\n' });
		while (curTime <= end && (freeWay || !stopOnObstacle)) {
			if (obstacleInFront()) {
				freeWay = false;
				robotSetLeds(127, 0);
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

	// TODO Add description
	public int getAngleToGoal(double x, double y) {
		int angle = (int) (Math.toDegrees(Math.atan2(y - Yg, x - Xg)));

		return reduceAngle(angle - Tg);
	}

	// TODO Add description
	public int getDistanceToGoal(double x, double y) {
		return (int) Math.sqrt(Math.pow(x - Xg, 2) + Math.pow(y - Yg, 2));
	}

	// TODO Add description
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

	// TODO Add description
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
		double dist = Math
				.min(getDistanceToGoal(x_ofIntersection, y_ofIntersection),
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
				turnRobotBalanced(90, dir);
				moveByVelocity(dist, false);
			}
			turnRobotBalanced(getAngleToGoal(0, 0), 'r');
			moveByVelocity(getDistanceToGoal(0, 0), false);
			turnRobotBalanced(Tg, 'l');
			robotSetLeds(127, 127);
			break;
		case 1:
			turnRobotBalanced(90, dir);
			Boolean freeway = true;
			for (int i = 0; i < 4; i++) {
				if (freeway && moveByVelocity(dist, true)) {
					turnRobotBalanced(90, dir);
				} else {
					freeway = false;
					ret = false;
				}
			}
			if (freeway) {
				turnRobotBalanced(getAngleToGoal(0, 0), 'r');
				moveByVelocity(getDistanceToGoal(0, 0), false);
				turnRobotBalanced(Tg, 'l');
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

		degrees = (int) (CorrFactAngleByDist * degrees);

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

	// TODO: Merge with obstacleLeft and obstacleRight
	/**
	 * methods tests whether an obstacle is in the range of the 3 front sensors
	 * or not
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

	// TODO Check if needed
	/**
	 * checks if an obstacle is in the range of the left sensor
	 * 
	 * @return
	 */
	private Boolean obstacleLeft() {
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

	// TODO Check if needed
	/**
	 * checks if an obstacle is in the range of the right sensor
	 * 
	 * @return
	 */
	private Boolean obstacleRight() {
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

			angle = getAngleToGoal(x, y);
			dist = (int) Math.sqrt(Math.pow(x - Xg, 2) + Math.pow(y - Yg, 2));

			writeLog("Moving to goal at angle " + angle + " in " + dist
					+ "cm distance");

			turnRobotBalanced(angle, 'r');
			robotSetLeds(127, 0);
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
				robotSetLeds(0, 127);
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

		turnRobotBalanced(theta - Tg, 'r');
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

			angle = getAngleToGoal(x, y);
			dist = getDistanceToGoal(x, y);

			writeLog("Moving to goal at angle " + angle + " in " + dist
					+ "cm distance");

			turnRobotBalanced(angle, 'r');
			robotSetLeds(0, 127);
			if (!moveByVelocity(dist, true)) {
				obstacleFound = true;
				robotSetLeds(127, 0);
				writeLog("Obstacle found at " + getMyPosition());
			}

			if (obstacleFound) {
				turnRobotBalanced((int) Math.signum((Math.random() - 0.5))
						* (90 + (int) (Math.random() * 20)), 'r');
				moveByVelocity(50, true);
			}
			if (getDistanceToGoal(x, y) < TOL) {
				goalReached = true;
			}
		}

		turnRobotBalanced(theta - Tg, 'r');
		robotSetLeds(127, 127);
	}

	/**
	 * 
	 * @return Position object based on current position
	 */
	public Position getMyPosition() {
		Position myPos = new Position(Xg, Yg, Tg);
		return myPos;
	}

	public void MoveToTarget(double x, double y, double theta) {
		int angle = 0, dist, moved, stepLength = 5;
		Boolean goalReached = false;
		int TOL = 3;
		while (!goalReached) {

			angle = getAngleToGoal(x, y);
			dist = getDistanceToGoal(x, y);

			writeLog("Moving to goal at angle " + angle + " in " + dist
					+ "cm distance");

			turnRobotBalanced(angle, 'r');
			robotSetLeds(127, 0);
			moveByVelocity(dist, false);
			if (getDistanceToGoal(x, y) < TOL) {
				goalReached = true;
			}
		}
		turnRobotBalanced((int)theta - Tg, 'r');
		robotSetLeds(127, 127);
	}
}
