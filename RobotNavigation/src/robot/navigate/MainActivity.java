package robot.navigate;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import jp.ksksue.driver.serial.FTDriver;
import android.app.Activity;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.TextView;
import at.ac.uibk.robotwasd.R;
import at.ac.uibk.robotwasd.R.id;
import at.ac.uibk.robotwasd.R.layout;
import at.ac.uibk.robotwasd.R.menu;

public class MainActivity extends Activity {

	@SuppressWarnings("unused")
	private String TAG = "iRobot";
	private TextView textLog;
	private FTDriver com;
	private Integer ObsDetecBorder = 50; // Working range of sensors is 10 to 80
											// cm (every other value should be
											// treated as no obstacle)

	private int Xg = 0, Yg = 0, Tg = 0;

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

		robotSetBar((byte) 50); // Initializing height of the bar.
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
		int i = 0;
		int n = 0;
		while (i < 3 || n > 0) {
			byte[] buffer = new byte[256];
			n = com.read(buffer);
			s += new String(buffer, 0, n);
			i++;
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
			// ignore
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

	// TODO Check if needed
	// TODO Add comment
	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		getMenuInflater().inflate(R.menu.main, menu);
		return true;
	}

	// TODO Check if needed
	// TODO Add comment
	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		switch (item.getItemId()) {
		case R.id.connect:
			connect();
			return true;

		case R.id.disconnect:
			disconnect();
			return true;

		default:
			return super.onOptionsItemSelected(item);
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
	// TODO Check, whether or not the robot does what it says in the function
	// description.
	public void robotSetLeds(int red, int blue) {
		writeLog(comReadWrite(new byte[] { 'u',
				(byte) Math.min(Math.max(red, 127), 0),
				(byte) Math.min(Math.max(blue, 127), 0), '\r', '\n' }));
	}

	public void robotSetVelocity(byte left, byte right) {
		writeLog(comReadWrite(new byte[] { 'i', left, right, '\r', '\n' }));
	}

	public void robotSetBar(byte value) {
		writeLog(comReadWrite(new byte[] { 'o', value, '\r', '\n' }));
	}

	// move forward
	public void buttonW_onClick(View v) {
		writeLog(comReadWrite(new byte[] { 'w', '\r', '\n' }));
	}

	// turn left
	public void buttonA_onClick(View v) {
		writeLog(comReadWrite(new byte[] { 'a', '\r', '\n' }));
	}

	// stop
	public void buttonS_onClick(View v) {
		writeLog(comReadWrite(new byte[] { 's', '\r', '\n' }));
	}

	// turn right
	public void buttonD_onClick(View v) {
		writeLog(comReadWrite(new byte[] { 'd', '\r', '\n' }));
	}

	// move backward
	public void buttonX_onClick(View v) {
		// logText(comReadWrite(new byte[] { 'x', '\r', '\n' }));
		robotSetVelocity((byte) -30, (byte) -30);
	}

	// lower bar a few degrees
	public void buttonMinus_onClick(View v) {
		writeLog(comReadWrite(new byte[] { '-', '\r', '\n' }));
	}

	// rise bar a few degrees
	public void buttonPlus_onClick(View v) {
		writeLog(comReadWrite(new byte[] { '+', '\r', '\n' }));
	}

	// fixed position for bar (low)
	public void buttonDown_onClick(View v) {
		robotSetBar((byte) 0);
	}

	// fixed position for bar (high)
	public void buttonUp_onClick(View v) {
		robotSetBar((byte) 255);
	}

	public void buttonLedOn_onClick(View v) {
		// logText(comReadWrite(new byte[] { 'r', '\r', '\n' }));
		robotSetLeds((byte) 255, (byte) 128);
	}

	public void buttonLedOff_onClick(View v) {
		// logText(comReadWrite(new byte[] { 'e', '\r', '\n' }));
		robotSetLeds((byte) 0, (byte) 0);
	}

	public void buttonSensor_onClick(View v) {
		Map<String, Integer> measurement = new HashMap<String, Integer>();
		Iterator<String> measureIter = measurement.keySet().iterator();

		measurement = getDistance();

		while (measureIter.hasNext()) {
			String key = measureIter.next();
			writeLog(measurement.get(key));
		}
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
	public void turnRobot(int angle, char dir) {
		// if (dir == 'r') {
		// angle = -angle;
		// }
		double corrAngle = 8.0 / 7; // Coming from a measurement
		int degrees = (int) (corrAngle * angle);
		int waitTimeMs = (1500 * degrees) / 90;
		writeLog(comReadWrite(new byte[] { 'l', (byte) degrees, '\r', '\n' },
				waitTimeMs));
		updateRotation(angle,dir);
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
					val = Integer.parseInt(value.substring(2, 4).toUpperCase(),
							16);

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
					writeLog("Integer parsing problem");
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
			movedDist++;
			measurement = getDistance();
			if (measurement.get("frontLeft") > ObsDetecBorder) {
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
}
