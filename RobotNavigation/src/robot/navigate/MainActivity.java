package robot.navigate;

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
	private Integer ObsDetecBorder = 10; // Working range of sensors is 10 to 80
										 // cm (every other value should be
										 // treated as no obstacle)


	/**
	 * Connects to the robot when app is started and initializes the position of the robot's bar.
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
		
		robotSetBar((byte) 0.75); // Initializing height of the bar. 
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
			writeLog("comWrite(data)\n"); // TODO Add the content of data to the log.
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
		while (i < 3 && n == 0) {
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
	 * @param data to write
	 * @return answer from serial interface
	 */
	public String comReadWrite(byte[] data) {
		return comReadWrite(data, 300);
	}
	

	/**
	 * Write data to serial interface, wait for the specified time and read answer.
	 * 
	 * @param data to write
	 * @param time to wait in ms
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
	 * Add the given text to the log file; also writes the length of the text into the log-file.
	 * @param text
	 */
	public void writeLog(String text) {
		if (text.length() > 0) {
			textLog.append("[" + text.length() + "] " + text + "\n");
		}
	}

	/**
	 * Add the given integer to the log file.
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
	 * @param red sets the intensity of the red colored LED (min value 0, max value 127)
	 * @param blue sets the intensity of the blue colored LED (min value 0, max value 127)
	 */
	// TODO Check, whether or not the robot does what it says in the function description.
	public void robotSetLeds(int red, int blue) {
		writeLog(comReadWrite(new byte[] { 'u', (byte) Math.min(Math.max(red, 127),0), (byte) Math.min(Math.max(blue, 127),0), '\r', '\n' }));
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
		// logText(comReadWrite(new byte[] { 'q','\r', '\n' }));
		// logText(Integer.parseInt(comReadWrite(new byte[] { 'q', '\r', '\n'
		// })));
		turn90onPlace('l');
		turn90onPlace('l');
		turn90onPlace('l');
		turn90onPlace('l');
		// MoveSquare(20,'r');
		// driveAndRead();
		// driveAroundNextCorner();
	}

	public void moveRobot(int dist) {
		double corrDistFact = 100.0 / 72; // Coming from a measurement
		int corrDist = (int) (dist * corrDistFact);
		while (Math.abs(corrDist) > 127) { // Byte stores values from -128 to
											// 127
			corrDist -= (int) (Math.signum(corrDist)) * 127;
			writeLog(comReadWrite(new byte[] { 'k', (byte) 127, '\r', '\n' },
					127 * 100));
		}
		writeLog(comReadWrite(new byte[] { 'k', (byte) corrDist, '\r', '\n' },
				Math.abs(dist) * 100));
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
			turn90onPlace(dir);
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
	public void turnRobot(byte angle, char dir) {
		int corrAngle = 1;
		int degrees = corrAngle * angle;
		int waitTimeMs = (500 * degrees) / 90;
		comReadWrite(new byte[] { (byte) dir, (byte) degrees, 'r', '\n' },
				waitTimeMs);
	}

	// test velocity values for left and right wheel
	public void turn90onPlace(char dir) {
		int left = 0, right = 0;
		switch (dir) {
		case 'l':
			left = 10;
			right = -10;
			break;
		case 'r':
			left = -10;
			right = 10;
			break;
		default:
			writeLog("wrong turn direction parameter");
			break;
		}
		double startTime = System.nanoTime();
		int runtime = 100; // we have to verify this value
		double finishTime = startTime + runtime;
		while (finishTime < System.nanoTime())
			comReadWrite(new byte[] { 'i', (byte) left, (byte) right, '\r',
					'\n' });
	}

	/**
	 * returns distance in cm
	 */
	public int getDistance() {
		return Integer.parseInt(comReadWrite(new byte[] { 'q', '\r', '\n' }));
	}

	/**
	 * Obstacle avoidance
	 * 
	 * drive on straight line and read sensor (check for obstacles)
	 */
	public void driveAndRead() {
		while (true) {
			moveRobot(5);
			if (getDistance() <= ObsDetecBorder) { // checks if robot hit near
													// obstacle
				moveAroundObstacle();
			}
		}
	}

	/**
	 * allows robot to drive around simple square object
	 */
	public void moveAroundObstacle() {
		int firEdge = 0, secEdge = 0; // length of each edge
		turn90onPlace('r');
		firEdge = driveAroundNextCorner();
		secEdge = driveAroundNextCorner();
		for (int i = firEdge; i > 0; i--) {
			moveRobot(5);
		}
		turn90onPlace('r');
	}

	/**
	 * move forward and turn left at next corner
	 */
	public int driveAroundNextCorner() {
		int movedDist = 0; // moved distance units
		boolean turnLeft = false;
		while (!turnLeft) {
			moveRobot(5);
			movedDist++;
			turn90onPlace('l');
			if (getDistance() > ObsDetecBorder) { // checks if robot can now
													// drive around obstacle
													// corner
				turnLeft = true;
			}
			turn90onPlace('r');
		}
		return movedDist;
	}

	public double[] getPosition() {
		return null;
	}

	/**
	 * Tries to move the robot to point (x,y)
	 * 
	 * first drive and read sensor (check for obstacles) method
	 */
	public void moveToPoint(int x, int y) {
		double tol = 1;
		double[] currLoc;
		int dist;
		int angle;
		int moved = 0;

		currLoc = getPosition();
		angle = (int) Math.atan((x - currLoc[0]) / (y - currLoc[1]));
		dist = (int) Math.sqrt(Math.pow(x - currLoc[0], 2)
				+ Math.pow(y - currLoc[1], 2));

		turnRobot((byte) angle, 'l');

		while (moved < dist) {
			moved++;
			driveAndRead();
		}
	}

}
