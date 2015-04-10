package robot.navigate;

import java.util.HashMap;
import java.util.Map;

import robot.generated.R;

import jp.ksksue.driver.serial.FTDriver;
import android.app.Activity;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.view.View;
import android.widget.TextView;

public class MainActivity extends Activity {

	private TextView textLog;
	private FTDriver com;
	private Integer ObsDetecBorder = 18; // Working range of sensors is 10 to 80
											// cm (every other value should be
											// treated as no obstacle)

	private int Xg = 0, Yg = 0, Tg = 0;

	// TODO: Fix Crash when robot is off

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

		com = new FTDriver((UsbManager) getSystemService(USB_SERVICE));

		connect();
		writeLog("onCreate!");
	}

	/**
	 * Establishes connection to the robot.
	 */
	public void connect() {
		if (com.begin(9600)) {
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
			writeLog("comWrite(data)");
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
			int i = 0;
			int n = 0;
			while (i < 3 || n > 0) {
				byte[] buffer = new byte[256];
				n = com.read(buffer);
				s += new String(buffer, 0, n);
				i++;
			}
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
		if (text.length() > 0) {
			textLog.append("[" + text.length() + "] " + text + "\n");
		}
	}

	/**
	 * Add the given integer to the log file.
	 * 
	 * @param value
	 */
	public void writeLog(int value) {
		textLog.append(value + "\n");
	}

	/**
	 * Add the given long integer to the log file.
	 * 
	 * @param value
	 */
	public void writeLog(long value) {
		textLog.append(value + "\n");
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

	// TODO: Check if needed
	// TODO: Add Comment
	public void moveRobotByVelocity(int left, int right, int time) {
		writeLog(comReadWrite(new byte[] { 'i', (byte) left, (byte) right,
				'\r', '\n' }));
		try {
			Thread.sleep(time);
		} catch (Exception e) {
		} finally {
			writeLog(comReadWrite(new byte[] { 's', '\r', '\n' }));
		}
	}

	// TODO: Delete, once it is not needed anymore
	// lower bar a few degrees
	public void buttonMinus_onClick(View v) {
		writeLog("Called by Minus");
		writeLog(comReadWrite(new byte[] { '-', '\r', '\n' }));
	}

	// TODO: Delete, once it is not needed anymore
	// rise bar a few degrees
	public void buttonPlus_onClick(View v) {
		writeLog("Called by Plus");
		writeLog(comReadWrite(new byte[] { '+', '\r', '\n' }));
	}

	public void buttonSensor_onClick(View v) {
		// TODO: Fix: Crashes when not connected.
		Map<String, Integer> measurement = new HashMap<String, Integer>();
		measurement = getDistance();
		for (Map.Entry<String, Integer> entry : measurement.entrySet()) {
			writeLog(entry.getKey() + entry.getValue());
		}
	}

	public void buttonDriveArObstacle_onClick(View v) {
		moveAroundObstacle();
	}

	public void buttonDriveAndRead_onClick(View v) {
		driveAndRead();
	}

	public void buttonDriveByVelo_onClick(View v) {
		driveByVelocity(50);
	}

	public void buttonBug1_onClick(View v) {
		bug1(120, 120);
	}

	// TODO: Delete once not needed anymore.
	public void buttonTest_onClick(View v) {
		try {
			turnRobot(90, 'l');
			Thread.sleep(500);
			turnRobot(90, 'r');
		} catch (Exception e) {
		}
	}

	public void driveByVelocity(int dist) {
		long start = System.nanoTime();
		writeLog("startTime: "+(int)start);
		double corrDistFact = 500.0; // Coming from a measurement
		long corrDist = (long) (dist * corrDistFact);
		long end = start + dist*corrDist;
		int waitTimeFact = 100, left = 20, right = 20;
		while (start <= end) {
			comReadWrite(new byte[] { 'i', (byte) left, (byte) right, '\r',
					'\n' }, waitTimeFact);
			start = System.nanoTime();
		}
		comReadWrite(new byte[] { 'i', (byte) 0, (byte) 0, '\r', '\n' },
				waitTimeFact);
	}

	/**
	 * updates global Position parameters after Robot moved one stepLength
	 * 
	 * @param stepLength
	 */

	public void updatePosition(int stepLength) {
		int movementX = 0, movementY = 0;
		movementX = (int) Math.sin(Tg) * stepLength;
		movementY = (int) (movementX / Math.tan(Tg));
		Xg = movementX;
		Yg = movementY;
		writeLog("my Position: (" + Xg + "," + Yg + "," + Tg + ")");
	}

	public void updateRotation(int angle, char dir) {
		switch (dir) {
		case 'l':
			Tg -= angle;
			break;
		case 'r':
			Tg += angle;
			break;
		default:
			writeLog("wrong input direction");
			break;
		}
		writeLog("my Position: (" + Xg + "," + Yg + "," + Tg + ")");
	}

	public void moveRobot(int dist) {
		double corrDistFact = 100.0 / 72; // Coming from a measurement
		int corrDist = (int) (dist * corrDistFact);
		int waitTimeFact = 100;
		while (Math.abs(corrDist) > 127) { // Byte stores values from -128 to
											// 127
			corrDist -= (int) (Math.signum(corrDist)) * 127;
			writeLog(comReadWrite(new byte[] { 'k', (byte) 127, '\r', '\n' },
					127 * waitTimeFact));
		}
		writeLog(comReadWrite(new byte[] { 'k', (byte) corrDist, '\r', '\n' },
				Math.abs(dist) * waitTimeFact));
		updatePosition(dist);
	}

	// TODO: Add Button to demonstrate
	/**
	 * tells the robot to move along a square
	 * 
	 * @param dir
	 *            ("l" = left; "r" = right)
	 * @param dist
	 *            in cm
	 */
	public void moveSquare(int dist, char dir) {
		for (int i = 0; i < 4; i++) {
			turnRobot(90, dir);
			moveRobot((byte) dist);
		}
	}

	/**
	 * tells the robot to turn
	 * 
	 * @param angle
	 *            in degrees
	 * @param dir
	 *            ("l" = left; "r" = right)
	 */
	public void turnRobot(int angle, char dir) {
		double corrAngle = 8.0 / 7; // Coming from a measurement
		int degrees = (int) (corrAngle * angle);
		int waitTimeMs = (1500 * degrees) / 90;

		if (dir == 'r') {
			degrees = -degrees;
		}
		writeLog(comReadWrite(new byte[] { 'l', (byte) degrees, '\r', '\n' },
				waitTimeMs));
		updateRotation(angle, dir);
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
				case 4:
					readSensor.put("frontLeft", val);
					break;
				case 5:
					readSensor.put("frontRight", val);
					break;
				case 8:
					readSensor.put("frontMiddle", val);
					break;
				}
				sensNr++;
			} catch (NumberFormatException e) {
				// Can't be parsed; do nothing
			}
		}
		return readSensor;
	}

	// TODO: Delete once not needed anymore
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
			if (getDistance().get("frontRight") <= ObsDetecBorder) { // checks
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
			writeLog(getDistance().get("frontRight"));
		}
		comReadWrite(new byte[] { 'i', 0, 0, '\r', '\n' });
	}

	
	// TODO Check if needed
	/**
	 * allows robot to drive around simple square object
	 */
	public void moveAroundObstacle() {
		int firEdge = 0, secEdge = 0; // length of each edge
		turnRobot(90, 'r');
		firEdge = driveAroundNextCorner();
		secEdge = driveAroundNextCorner();
		for (int i = firEdge; i > 0; i--) {
			moveRobot(5);
		}
		turnRobot(90, 'r');
	}

	// TODO: Delete once not needed anymore
	/**
	 * move forward and turn left at next corner
	 */
	public int driveAroundNextCorner() {
		int movedDist = 0; // moved distance units
		boolean turnLeft = false;
		Map<String, Integer> measurement = new HashMap<String, Integer>();
		while (!turnLeft) {
			moveRobot(5);
			movedDist = movedDist + 5;
			measurement = getDistance();
			writeLog(measurement.get("frontLeft"));
			if (measurement.get("frontLeft") > ObsDetecBorder) {
				writeLog("My way is free");
				turnLeft = true;
			}
		}
		turnRobot(90, 'l');
		return movedDist;
	}

	/**
	 * Tries to move the robot to point (x,y)
	 * 
	 * first drive and read sensor (check for obstacles) method
	 */
	public void moveToPoint(int x, int y) {
		int dist;
		int angle;
		int moved = 0;

		angle = (int) Math.atan((x - Xg) / (y - Yg));
		dist = (int) Math.sqrt(Math.pow(x - Xg, 2) + Math.pow(y - Yg, 2));

		// we need to update the robots own position information
		turnRobot((byte) angle, 'r');
		Tg = angle;

		Map<String, Integer> measurement = new HashMap<String, Integer>();
		while (moved < dist) {
			moved++;
			// we need to update the robots own position information after each
			// step
			// driveAndRead();
			int stepLength = 2;
			moveRobot(stepLength);
			measurement = getDistance();
			if (measurement.get("frontRight") <= ObsDetecBorder) {
				moveAroundObstacle();
			}

		}
	}

	// TODO: Check if needed
	public void moveToGoal(int x, int y) {
		int dist;
		int angle;
		int moved = 0;

		angle = (int) (360*Math.atan(((double) (x - Xg)) /  (y - Yg))/(2*Math.PI));
		dist = (int) Math.sqrt(Math.pow(x - Xg, 2) + Math.pow(y - Yg, 2));
		
		writeLog("Moving to goal at angle " + angle + " in " + dist + "cm distance");

		// we need to update the robots own position information
		turnRobot((byte) angle, 'r');
		Tg = angle;

		Map<String, Integer> measurement = new HashMap<String, Integer>();
		while (moved < dist) {
			moved++;
			int stepLength = 2;
			moveRobot(stepLength);
			measurement = getDistance();
			if (measurement.get("frontRight") <= ObsDetecBorder) {
				writeLog("Obstacle found at " + getMyPosition());
				roundObstacle(x, y);
				break;
			}

		}
	}

	// TODO: add comment
	public void roundObstacle(int goalX, int goalY) {
		Position startPosition = getMyPosition();
		Position closestPosition = startPosition;
		boolean startPositionReached = false;
		boolean closestPositionReached = false;
		final double TOL = 3.0; // Tolerated distance for comparison of current
								// and start position
		int movedTotalDistance = 0; // minimum distance needs to exceed a
									// certain value before current and start
									// position get compared

		int curGoalDist = (int) Math.sqrt(Math.pow(Xg - goalX, 2)
				+ Math.pow(Yg - goalY, 2)); // distance form current position to
											// goal
		int closestDistance = curGoalDist;
		
		writeLog("Going around obstacle");

		while (!startPositionReached) {
			while ((getDistance().get("frontLeft") < ObsDetecBorder)) { // Drive
																		// around
																		// obstacle
																		// and
																		// find
																		// closest
																		// position
																		// to
																		// goal
				if (getDistance().get("frontRight") < ObsDetecBorder) { // If
																			// there
																			// is
																			// an
																			// obstacle
																			// in
																			// front,
																			// turn
																			// right
																			// and
																			// continue
					turnRobot(90, 'r');
				}
				moveRobot(12);
				movedTotalDistance = movedTotalDistance + 1;
				curGoalDist = (int) Math.sqrt(Math.pow(Xg - goalX, 2)
						+ Math.pow(Yg - goalY, 2)); // distance form current
													// position to goal // TODO:
													// Implement function
				if (curGoalDist < closestDistance) { // Check whether current
														// point is closer to
														// goal or not
					closestDistance = curGoalDist;
					closestPosition = getMyPosition();
					writeLog("Closest point to goal found at (" + getMyPosition().x + "," + getMyPosition().y + ") - distance to goal: " + closestDistance + "cm");
				}
				if ((startPosition.minus(getMyPosition()) < TOL)
						&& movedTotalDistance >= 5) { // Check if start position
														// is reached again
					startPositionReached = true;
					writeLog("Back at starting position");
				}
			}
		}

		writeLog("Navigating to the closest point (" + closestPosition.x + "," + closestPosition.y + ")");
		while (!closestPositionReached) {
			while ((getDistance().get("frontLeft") > ObsDetecBorder)) { // Drive
																		// around
																		// obstacle
																		// and
																		// find
				// closest position to goal
				if (getDistance().get("frontRight") < ObsDetecBorder) { // If
																			// there
																			// is
																			// an
																			// obstacle
																			// in
																			// front,
																			// turn
																			// right
																			// and
																			// continue
					turnRobot(90, 'r');
				}
				moveRobot(12);
				if (closestPosition.minus(getMyPosition()) < TOL) {
					closestPositionReached = true;
					writeLog("Closest Point reached");
				}
			}
			turnRobot(90, 'l');
		}

		moveToGoal(goalX, goalY);
	}

	/**
	 * Bug1 algorithm 1) head toward goal 2) if an obstacle is encountered
	 * circumnavigate it and remember how close you get to the goal 3) return to
	 * that closest point(by wall-following) and continue
	 */
	public void bug1(int x, int y) {
		moveToGoal(x, y);
	}

	public Position getMyPosition() {
		Position myPos = new Position(Xg, Yg, Tg);
		return myPos;
	}
}
