package at.ac.uibk.robotwasd;

import jp.ksksue.driver.serial.FTDriver;
import android.app.Activity;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.TextView;

public class MainActivity extends Activity {

	@SuppressWarnings("unused")
	private String TAG = "iRobot";
	private TextView textLog;
	private FTDriver com;
	private Integer ObsDetecBorder = 5;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);

		textLog = (TextView) findViewById(R.id.textLog);

		com = new FTDriver((UsbManager) getSystemService(USB_SERVICE));

		connect();
	}

	public void connect() {
		// TODO implement permission request

		if (com.begin(9600)) {
			textLog.append("connected\n");
		} else {
			textLog.append("could not connect\n");
		}
	}

	public void disconnect() {
		com.end();
		if (!com.isConnected()) {
			textLog.append("disconnected\n");
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
			textLog.append("not connected\n");
		}
	}

	/**
	 * reads from the serial buffer. due to buffering, the read command is
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
	 * write data to serial interface, wait 100 ms and read answer.
	 * 
	 * @param data
	 *            to write
	 * @return answer from serial interface
	 */
	public String comReadWrite(byte[] data) {
		com.write(data);
		try {
			Thread.sleep(3000);
		} catch (InterruptedException e) {
			// ignore
		}
		return comRead();
	}

	public String comReadWrite(byte[] data, int time) {
		com.write(data);
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
			// ignore
		}
		return comRead();
	}

	public void logText(String text) {
		if (text.length() > 0) {
			textLog.append("[" + text.length() + "] " + text + "\n");
		}
	}

	public void logText(int value) {
		textLog.append(value + "\n");
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		getMenuInflater().inflate(R.menu.main, menu);
		return true;
	}

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

	public void robotSetLeds(byte red, byte blue) {
		logText(comReadWrite(new byte[] { 'u', red, blue, '\r', '\n' }));
	}

	public void robotSetVelocity(byte left, byte right) {
		logText(comReadWrite(new byte[] { 'i', left, right, '\r', '\n' }));
	}

	public void robotSetBar(byte value) {
		logText(comReadWrite(new byte[] { 'o', value, '\r', '\n' }));
	}

	// move forward
	public void buttonW_onClick(View v) {
		logText(comReadWrite(new byte[] { 'w', '\r', '\n' }));
	}

	// turn left
	public void buttonA_onClick(View v) {
		logText(comReadWrite(new byte[] { 'a', '\r', '\n' }));
	}

	// stop
	public void buttonS_onClick(View v) {
		logText(comReadWrite(new byte[] { 's', '\r', '\n' }));
	}

	// turn right
	public void buttonD_onClick(View v) {
		logText(comReadWrite(new byte[] { 'd', '\r', '\n' }));
	}

	// move backward
	public void buttonX_onClick(View v) {
		// logText(comReadWrite(new byte[] { 'x', '\r', '\n' }));
		robotSetVelocity((byte) -30, (byte) -30);
	}

	// lower bar a few degrees
	public void buttonMinus_onClick(View v) {
		logText(comReadWrite(new byte[] { '-', '\r', '\n' }));
	}

	// rise bar a few degrees
	public void buttonPlus_onClick(View v) {
		logText(comReadWrite(new byte[] { '+', '\r', '\n' }));
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

	public void moveRobot(byte dist) {
		int correctDist = 1;
		int fdist = dist * correctDist;
		logText(comReadWrite(new byte[] { 'k', (byte) fdist, '\r', '\n' },
				dist * 100));
	}
	
	public void moveRobot(int dist) {
		int correctDist = 1;
		int fdist = dist * correctDist;
		logText(comReadWrite(new byte[] { 'k', (byte) fdist, '\r', '\n' },
				dist * 100));
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
			left = 100;
			right = -100;
			break;
		case 'r':
			left = -100;
			right = 100;
			break;
		default:
			logText("wrong turn direction parameter");
			break;
		}
		comReadWrite(new byte[] { 'i', (byte) left, (byte) right, '\r', '\n' });
	}

	/**
	 * tells the robot to move along a square
	 * 
	 * @param dir
	 *            ("l" = left; "r" = right)
	 * @param dist
	 *            in cm
	 */
	public void MoveSquare(int dist, char dir) {
		for (int i = 0; i < 4; i++) {
			turnRobot((byte) 90, dir);
			moveRobot((byte) dist);
		}
	}

	public void buttonSensor_onClick(View v) {
		// logText(comReadWrite(new byte[] { 'q','\r', '\n' }));
		logText(Integer.parseInt(comReadWrite(new byte[] { 'q', '\r', '\n' })));
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
			if (getDistance() <= ObsDetecBorder) {	//checks if robot hit near obstacle (value of 5 is randomly chosen)
				moveAroundObstacle();
			}
		}
	}
	
	/**
	 * allows robot to drive around simple square object
	 */
	public void moveAroundObstacle() {
		int firEdge = 0,secEdge=0;				
		turn90onPlace('r');
		firEdge = driveAroundNextCorner();
		secEdge = driveAroundNextCorner();
		for(int i=firEdge;i>0;i--){
			moveRobot(5);
		}
		turn90onPlace('r');
	}
		
	/**
	 * move forward and turn left at next corner 
	 */
	public int driveAroundNextCorner(){
		int movedDist = 0;							//moved distance units
		boolean turnLeft = false;
		while(!turnLeft){
			moveRobot(5);
			movedDist++;
			turn90onPlace('l');
			if(getDistance() > ObsDetecBorder){ 	//checks if robot can now drive around obstacle corner (value of 66 isn't correct)
				turnLeft = true;
			} 
			turn90onPlace('r');
		}
		return movedDist;
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
		angle = (int) Math.atan((x-currLoc[0])/(y-currLoc[1]));
		dist = (int) Math.sqrt(Math.pow(x-currLoc[0],2) + Math.pow(y-currLoc[1], 2));
		
		turnRobot((byte) angle,'l');
		
		while (moved < dist) {
			moved++;
			driveAndRead();
		}
	}
	
}
