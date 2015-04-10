package robot.navigate;

import java.util.HashMap;
import java.util.Map;

import robot.generated.R;

import org.apache.http.impl.conn.Wire;

import jp.ksksue.driver.serial.FTDriver;
import android.app.Activity;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.view.View;
import android.widget.TextView;

public class MainActivity extends Activity {

	private TextView textLog;
	private FTDriver com;
	private Integer ObsDetecBorder = 50; // Working range of sensors is 10 to 80
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

		robotSetBar((byte) 127); // TODO Still drops to the floor.
		writeLog("onCreate!\n");
	}

	/**
	 * Establishes connection to the robot.
	 */
	public void connect() {
		if (com.begin(9600)) {
			writeLog("connected\n");
		} else {
			writeLog("could not connect\n");
		}
	}

	/**
	 * Closes the connection to the robot.
	 */
	public void disconnect() {
		com.end();
		if (!com.isConnected()) {
			writeLog("disconnected\n");
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
			writeLog("comWrite(data)\n"); // TODO Add the content of data to the
											// log.
			writeLog(data.toString());
		} else {
			writeLog("not connected\n");
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
			writeLog("not connected\n");
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
				(byte) Math.max(Math.min(blue, 127), 0), 
				'\r', 
				'\n' }));
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

	public void robotSetVelocity(byte left, byte right) {
		writeLog(comReadWrite(new byte[] { 'i', left, right, '\r', '\n' }));
	}

	public void robotSetBar(byte value) {
		writeLog(comReadWrite(new byte[] { 'o', value, '\r', '\n' }));
	}

	// move forward
	public void buttonW_onClick(View v) {
		writeLog("Called by W");
		writeLog(comReadWrite(new byte[] { 'w', '\r', '\n' }));
	}

	// turn left
	public void buttonA_onClick(View v) {
		writeLog("Called by A");
		writeLog(comReadWrite(new byte[] { 'a', '\r', '\n' }));
	}

	// stop
	public void buttonS_onClick(View v) {
		writeLog("Called by S");
		writeLog(comReadWrite(new byte[] { 's', '\r', '\n' }));
	}

	// turn right
	public void buttonD_onClick(View v) {
		writeLog("Called by D");
		writeLog(comReadWrite(new byte[] { 'd', '\r', '\n' }));
	}

	// move backward
	public void buttonX_onClick(View v) {
		writeLog("Called by X");
		// logText(comReadWrite(new byte[] { 'x', '\r', '\n' }));
		robotSetVelocity((byte) -30, (byte) -30);
	}

	// lower bar a few degrees
	public void buttonMinus_onClick(View v) {
		writeLog("Called by Minus");
		writeLog(comReadWrite(new byte[] { '-', '\r', '\n' }));
	}

	// rise bar a few degrees
	public void buttonPlus_onClick(View v) {
		writeLog("Called by Plus");
		writeLog(comReadWrite(new byte[] { '+', '\r', '\n' }));
	}

	// fixed position for bar (low)
	public void buttonDown_onClick(View v) {

		turnRobot(90, 'l');
		turnRobot(90, 'r');

		robotSetBar((byte) 0);
	}

	// fixed position for bar (high)
	public void buttonUp_onClick(View v) {
		writeLog("Called by Up");
		robotSetBar((byte) 255);
	}

	public void buttonLedOn_onClick(View v) {
		writeLog("Called by LED ON");
		// logText(comReadWrite(new byte[] { 'r', '\r', '\n' }));
		try {
			Thread.sleep(500);
			robotSetLeds((byte) 50, (byte) 50);
			Thread.sleep(500);
			robotSetLeds((byte) 127, (byte) 50);
			Thread.sleep(500);
			robotSetLeds((byte) 127, (byte) 0);
			Thread.sleep(500);
			robotSetLeds((byte) 50, (byte) 50);
			Thread.sleep(500);
			robotSetLeds((byte) 50, (byte) 127);
			Thread.sleep(500);
			robotSetLeds((byte) 0, (byte) 127);
			Thread.sleep(500);
			robotSetLeds((byte) 0, (byte) 0);
		} catch (Exception e) {
		}
	}

	public void buttonLedOff_onClick(View v) {
		writeLog("Called by LedOff");
		// logText(comReadWrite(new byte[] { 'e', '\r', '\n' }));
		moveRobotByVelocity(100, 100, 1000);
	}

	public void buttonSensor_onClick(View v) {
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
		bug1();
	}

	public void buttonTest_onClick(View v) {
		try {
			robotSetLeds(0,127);
			Thread.sleep(500);
			robotSetLeds(0,50);
			Thread.sleep(500);
			robotSetLeds(0,127);
			Thread.sleep(500);
			robotSetLeds(50,127);
			Thread.sleep(500);
			robotSetLeds(127,127);
			Thread.sleep(500);
			robotSetLeds(127,50);
			Thread.sleep(500);
			robotSetLeds(127,0);
			Thread.sleep(500);
			robotSetLeds(0,0);
		} catch (Exception e) {
		}
	}
	
	public void driveByVelocity(int dist) {
		long start = System.nanoTime();
		writeLog((int)start);
		// double corrDistFact = 100.0 / 72; // Coming from a measurement
		// long corrDist = (long) (dist * corrDistFact);
		long end = start + 2000;
		writeLog((int)end);
		int waitTimeFact = 100, left = 20, right = 20;
		while (start <= end) {
			comReadWrite(new byte[] { 'i', (byte) left, (byte) right, '\r',
					'\n' }, waitTimeFact);
		}
		comReadWrite(new byte[] { 'i', (byte) 0, (byte) 0, '\r',
		'\n' }, waitTimeFact);
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
	// TODO Allow to turn right.
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
	 * returns distance in cm
	 */
	public Map<String, Integer> getDistance() {
		int i = 0;
		int sensNr;
		int val;
		String[] sensorInfo;
		Map<String, Integer> readSensor = new HashMap<String, Integer>();
		while (i < 1) {
			i++;
			try {
				Thread.sleep(0);
			} catch (InterruptedException e) {
				e.getMessage();
			}
			sensorInfo = comReadWrite(new byte[] { 'q', '\r', '\n' })
					.split(" ");
			sensNr = 1;
			for (String value : sensorInfo) {
				try {
					val = Integer.parseInt(value.substring(2, 4), 16);

					switch (sensNr) {
					case 4:
						readSensor.put("frontLeft", val);
						break;
					case 8:
						readSensor.put("frontMiddle", val);
						break;
					case 5:
						readSensor.put("frontRight", val);
						break;
					}
					sensNr++;
				} catch (NumberFormatException e) {
					// Can't be parsed; do nothing
				}
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
			// try {
			// Thread.sleep(200);
			// } catch (InterruptedException e) {
			// writeLog("Can't sleep");
			// }
			try {
				Thread.sleep(50);
			} catch (Exception e) {
			}
			if (getDistance().get("frontMiddle") <= ObsDetecBorder) { // checks
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
	public void moveToPoint(int x, int y, int theta) {
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
			if (measurement.get("frontMiddle") <= ObsDetecBorder) {
				moveAroundObstacle();
			}

		}
	}

	public void moveToGoal(int x, int y) {
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
			int stepLength = 2;
			moveRobot(stepLength);
			measurement = getDistance();
			if (measurement.get("frontMiddle") <= ObsDetecBorder) {
				roundObstacle(x, y);
				break;
			}

		}
	}

	public void roundObstacle(int Goalx, int Goaly) {
		HashMap<Integer, Position> goalDist = new HashMap<Integer, Position>();
		// distance form current postition to goal
		int curGoalDist = (int) Math.sqrt(Math.pow(Xg - Goalx, 2)
				+ Math.pow(Yg - Goaly, 2));
		goalDist.put(curGoalDist, getMyPosition());
		turnRobot(90, 'r');
		for (int i = 0; i < 4; i++) {
			boolean turnLeft = false;
			Map<String, Integer> measurement = new HashMap<String, Integer>();
			while (!turnLeft) {
				moveRobot(5);
				curGoalDist = (int) Math.sqrt(Math.pow(Xg - Goalx, 2)
						+ Math.pow(Yg - Goaly, 2));
				goalDist.put(curGoalDist, getMyPosition());
				if (goalDist.containsKey(getMyPosition())) {
					break; // robot drove around obstacle and measurte all
							// distances
				}
				measurement = getDistance();
				writeLog(measurement.get("frontLeft"));
				if (measurement.get("frontLeft") > ObsDetecBorder) {
					writeLog("My way is free");
					turnLeft = true;
				}
			}
			turnRobot(90, 'l');
			int minDist = curGoalDist;
			Position nextStartPt = null;
			for (Integer elem : goalDist.keySet()) {
				if (elem < minDist) {
					minDist = elem;
					nextStartPt = goalDist.get(elem);
				}
			}
			moveToPoint(nextStartPt.getX(), nextStartPt.getX(),
					nextStartPt.getTheta());
			moveToGoal(Goalx, Goaly);
		}
		turnRobot(90, 'r');
	}

	/**
	 * Bug1 algorithm 1) head toward goal 2) if an obstacle is encountered
	 * circumnavigate it and remember how close you get to the goal 3) return to
	 * that closest point(by wall-following) and continue
	 */
	public void bug1() {
		int[] goal = null;
		goal[0] = 30;
		goal[1] = 30;
		moveToGoal(goal[0], goal[1]);
	}

	public Position getMyPosition() {
		Position myPos = new Position(Xg, Yg, Tg);
		return myPos;
	}
}
