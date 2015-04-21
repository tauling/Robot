package robot.navigate;

import java.util.HashMap;
import java.util.Map;

import robot.generated.R;

import jp.ksksue.driver.serial.FTDriver;
import android.app.Activity;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.text.method.ScrollingMovementMethod;
import android.view.View;
import android.widget.ScrollView;
import android.widget.TextView;

public class MainActivity extends Activity {

	// TODO: Create a class "Robot" with public and private methods
	
	// TODO: Working Demos -> rename to Examination Task 1
	
	// TODO: Fix: (x,y) are switched in move to goal methods
	
	// TODO: Change orientation; (0,0,0) means, that the robot is facing to the right

	// TODO: Allow the user to enter values in the app.

	// TODO: Add button to stop all threads

	// TODO: Add a function that allows to drive curves (and updates odometry)

	// TODO: Detect green and red blobs

	// TODO: Detect a ball and calculate it's lowest position (where it touches
	// the surface)

	// TODO: Move to the ball and use the robot's cage to catch it.

	// TODO: Detect multiple balls at the same time

	// TODO: Also detect blue, yellow, black and white blobs
	
	// TODO: implement MoveToTarget(x,y,theta) (ignoring obstacles)

	// TODO: explore workspace 

	// TODO: catch ball -> drive to target corner

	private TextView textLog;
	
	protected Robot robot;
	
	
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
		textLog.setMovementMethod(new ScrollingMovementMethod());
		ScrollView svLog = (ScrollView) findViewById(R.id.scrollMe);

		FTDriver com = new FTDriver((UsbManager) getSystemService(USB_SERVICE));
		
		robot = new Robot(textLog, svLog, com);
		robot.connect();
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

	public void button360Deg_onClick(View v) {
		Thread t = new Thread() {

			@Override
			public void run() {
				robot.turnRobot(180, 'r');
				robot.turnRobot(180, 'r');
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
				robot.robotSetLeds(0,0);
				robot.robotSetLeds(127,127);
				robot.robotSetLeds(0,0);
				robot.robotSetLeds(127,127);
				robot.robotSetLeds(0,0);
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
				
			};
		};

		t.start();
	}

	public void buttonTest2_onClick(View v) {

		Thread t = new Thread() {

			@Override
			public void run() {
				
			};
		};

		t.start();
	}

}
