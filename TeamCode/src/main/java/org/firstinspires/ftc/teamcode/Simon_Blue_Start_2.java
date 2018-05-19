package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import android.graphics.Color;

//IMU Imports
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="SIMON: Blue_Start_2", group="SIMON")

public class Simon_Blue_Start_2 extends LinearOpMode {
	private DcMotor front_Left;
	private DcMotor front_Right;
	private DcMotor rear_Left;
	private DcMotor rear_Right;
	private DcMotor front_Lift;
	private DcMotor rear_Lift;
	private Servo colour_arm;
	private DcMotor relic_Slide;
	private Servo gripper_1;
	private Servo gripper_2;
	private Servo gripper_3;
	private Servo jewel_Flick;
	private ColorSensor sensorColour;
	private ElapsedTime	 runtime = new ElapsedTime();
	
		//declare variables for gripper control

	double Gripper1_Init = 0.35; // Start point of gripper 1
	double Gripper2_Init = 0.45; // Start point of gripper 2
	double Gripper3_Init = -0.9; // Start point of gripper 3, Relic
	double ColourArm_Init = 0.64; // Start point of gripper 2
	double Flickr_Init = 0.5; //  Start point for the jewel flick servo
	double Flickr_Fwd = 0.1; // Jewel Flick Forward position
	double Flickr_Rev = 0.9; // Jewel Flick Revers position


	//IMU Declarations	
	private BNO055IMU imu;
	private Orientation lastAngles = new Orientation();
	double globalAngle, power = .30, correction;
	
	
	@Override
	public void runOpMode() {
		front_Left = hardwareMap.get(DcMotor.class, "Front_Left");
		front_Right = hardwareMap.get(DcMotor.class, "Front_Right");
		rear_Left = hardwareMap.get(DcMotor.class, "Rear_Left");
		rear_Right = hardwareMap.get(DcMotor.class, "Rear_Right");
		front_Lift = hardwareMap.get(DcMotor.class, "Front_Lift");
		rear_Lift = hardwareMap.get(DcMotor.class, "Rear_Lift");
		colour_arm = hardwareMap.get(Servo.class, "colour_arm");
		relic_Slide = hardwareMap.get(DcMotor.class, "Relic_Slide");
		gripper_1 = hardwareMap.get(Servo.class, "Gripper_1");
		gripper_2 = hardwareMap.get(Servo.class, "Gripper_2");
		gripper_3 = hardwareMap.get(Servo.class, "Gripper_3");
		jewel_Flick = hardwareMap.get(Servo.class, "Flickr");

		sensorColour = hardwareMap.get(ColorSensor.class, "colour");



//Set the servos to the Init Positions
		gripper_1.setPosition(Gripper1_Init);
		gripper_2.setPosition(Gripper2_Init);
		gripper_3.setPosition(Gripper3_Init);
		colour_arm.setPosition(ColourArm_Init);
		jewel_Flick.setPosition(Flickr_Init);

//For the alliance colour set it here.
		String Alliance = "Blue";
		telemetry.addData("Alliance", Alliance );
		telemetry.update();
		
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

		parameters.mode				= BNO055IMU.SensorMode.IMU;
		parameters.angleUnit		   = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit		   = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.loggingEnabled	  = false;


		imu = hardwareMap.get(BNO055IMU.class, "imu");

		imu.initialize(parameters);

		telemetry.addData("Mode", "calibrating...");
		telemetry.update();

		// make sure the imu gyro is calibrated before continuing.
		while (!isStopRequested() && !imu.isGyroCalibrated())
		{
			sleep(50);
			idle();
		}

		telemetry.addData("Mode", "waiting for start");
		telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
		telemetry.update();
		
		telemetry.addData("Status", "Initialized");
		telemetry.update();
		// Wait for the game to start (driver presses PLAY)
		waitForStart();

		// run until the end of the match (driver presses STOP)
		if (opModeIsActive()) {
		// To grip the glyph we need to extend the arm, close gripper and lift
			//Gripper Close
			gripper_1.setPosition(0.76);
			gripper_2.setPosition(0.125);
			sleep(500)	;
			front_Lift.setPower(1);
			sleep(2000);
			front_Lift.setPower(0);
			sleep(250);

// Lower the colour arm, test for the colour detected and determine which way to flick
			knockJewel(Alliance);
			colour_arm.setPosition(ColourArm_Init);
			jewel_Flick.setPosition(Flickr_Init);
			sleep(1000);

// Now move off the ramp to the cryptolocker
			encoderDrive(0.15, 550, 1, 6); // Reverse off balance Board
			sleep(250);
			turn(180); // Turn 180 Degrees to face CrypotoBox
			encoderDrive(0.3, 340, 2, 4 ); // Move Sideways to line up
			sleep(250);
			encoderDrive(0.3, 350, 0, 4);
			sleep(250);
			gripper_1.setPosition(Gripper1_Init);
			gripper_2.setPosition(Gripper2_Init);
			sleep(250);
			encoderDrive(0.3, 50, 1 ,2); // Reverse back from cryptobox

			// gripper_1.setPosition(0.3);
			// gripper_2.setPosition(0.3);
			// encoderDrive(0.3, 0.3, -20);
				
			// telemetry.addData("Degrees:", gyro.getIntegratedZValue());
			// telemetry.update();
		}	
	}		



//*******************************End of the autonoumous code, after here come the Methods used**********************************			

	//Drives the robot a set distance in MM, at a certain Speed
	private void encoderDrive(double Speed, double MM, int Direction, double timeoutS) {
	//Direction is passed in as 0-FWD, 1-REV, 2-Right, 3-Left


		final double COUNTS_PER_MOTOR_REV = 1440;	// eg: TETRIX Motor Encoder
		final double DRIVE_GEAR_REDUCTION = 1.0;	 // This is < 1.0 if geared UP
		final double WHEEL_DIAMETER_MM = 100;	 // For figuring circumference, this isn't what was measured, but it what works
		final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
				(WHEEL_DIAMETER_MM * 3.1415); // Works out the Encoder Counts per MM

		runtime.reset();

		//initialise some variables for the subroutine
		int newLeftTarget =0;
		int newRightTarget =0;
		// Determine new target position, and pass to motor controller we only do this in case the encoders are not totally zero'd
		if ((Direction == 0) || (Direction ==2)){
			newLeftTarget = rear_Left.getCurrentPosition() + (int) (MM * COUNTS_PER_MM * 0.602);
			newRightTarget = rear_Right.getCurrentPosition() + (int) (MM * COUNTS_PER_MM * 0.602);
		}
		if ((Direction == 1) || (Direction == 3)){
			newLeftTarget = rear_Left.getCurrentPosition() - (int) (MM * COUNTS_PER_MM * 0.678);
			newRightTarget = rear_Right.getCurrentPosition() - (int) (MM * COUNTS_PER_MM * 0.678);
		}

		if (Direction == 0) {
			while ((rear_Left.getCurrentPosition() < newLeftTarget) && (rear_Right.getCurrentPosition() < newRightTarget) && opModeIsActive() && (runtime.seconds() < timeoutS)){
				// write the values to the motors
				front_Right.setPower(Speed);
				front_Left.setPower(-Speed);
				rear_Left.setPower(-Speed);
				rear_Right.setPower(Speed);
				
			}
			front_Left.setPower(0);
			front_Right.setPower(0);
			rear_Left.setPower(0);
			rear_Right.setPower(0);

		}
		if (Direction == 1) { // REV
			while ((rear_Left.getCurrentPosition() > newLeftTarget) && (rear_Right.getCurrentPosition() > newRightTarget) && opModeIsActive() && (runtime.seconds() < timeoutS)){
				front_Right.setPower(-Speed);
				front_Left.setPower(Speed);
				rear_Left.setPower(Speed);
				rear_Right.setPower(-Speed);
			}
			front_Left.setPower(0);
			front_Right.setPower(0);
			rear_Left.setPower(0);
			rear_Right.setPower(0);

		}
		
		if (Direction == 2) { // Right
			while ((rear_Left.getCurrentPosition() < newLeftTarget) && (rear_Right.getCurrentPosition() < newRightTarget) && opModeIsActive() && (runtime.seconds() < timeoutS)){
				front_Right.setPower(-Speed);
				front_Left.setPower(-Speed);
				rear_Left.setPower(Speed);
				rear_Right.setPower(Speed);
			}
			front_Left.setPower(0);
			front_Right.setPower(0);
			rear_Left.setPower(0);
			rear_Right.setPower(0);

		}

		if (Direction == 3) { // Left
			while ((rear_Left.getCurrentPosition() > newLeftTarget) && (rear_Right.getCurrentPosition() > newRightTarget) && opModeIsActive() && (runtime.seconds() < timeoutS)){
				front_Right.setPower(Speed);
				front_Left.setPower(Speed);
				rear_Left.setPower(-Speed);
				rear_Right.setPower(-Speed);
			}
			front_Left.setPower(0);
			front_Right.setPower(0);
			rear_Left.setPower(0);
			rear_Right.setPower(0);

		}
		
		

			//Shows the target to reach, and what the motors finished at in reality
			telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
			telemetry.addData("Path2", "Running at %7d :%7d", rear_Left.getCurrentPosition(), rear_Right.getCurrentPosition());
			telemetry.update();
		}




	private void turn(int degreesNeeded){
		
	 	while ((Math.abs(getAngle()) <= degreesNeeded) && opModeIsActive()){
	 		if (degreesNeeded - Math.abs(getAngle()) > 60){
				front_Right.setPower(-0.3);
				front_Left.setPower(-0.3);
				rear_Left.setPower(-0.3);
				rear_Right.setPower(-0.3);
	 		}else{
				front_Right.setPower(-0.08);
				front_Left.setPower(-0.08);
				rear_Left.setPower(-0.08);
				rear_Right.setPower(-0.08);
	 		}
 			telemetry.addData("Degrees:", getAngle());
	 		telemetry.update();
	 		}
		front_Left.setPower(0);
		front_Right.setPower(0);
		rear_Left.setPower(0);
		rear_Right.setPower(0);

	}
	
	// Senses the colour in front of it and returns whether it is RED or BLUE
	private int colourPicker() {
		float hsvValues[] = {0F, 0F, 0F};
		final float values[] = hsvValues;
		final double SCALE_FACTOR = 255;
		// convert the RGB values to HSV values.
		// multiply by the SCALE_FACTOR.
		// then cast it back to int (SCALE_FACTOR is a double)
		Color.RGBToHSV((int) (this.sensorColour.red() * SCALE_FACTOR),
				(int) (sensorColour.green() * SCALE_FACTOR),
				(int) (sensorColour.blue() * SCALE_FACTOR),
				hsvValues);
				telemetry.addData("Hue", hsvValues[0]);
				telemetry.update();
				// return the colour detected.
		if (hsvValues[0] > 340 || hsvValues[0] < 20) {
			return Color.RED;
		} else if (hsvValues[0] > 180 || hsvValues[0] < 290) {
			return Color.BLUE;
		}
		return 0;
	}



	private void knockJewel(String alliance){
//Don't forget to set the Alliance Colour String at the begining
		//Lower the colour arm.
		colour_arm.setPosition(0.6);
		sleep(200); // Allow a bit of time for the colour sensor
		colour_arm.setPosition(0.1);

		sleep(2000); // Allow a bit of time for the colour sensor
		if (colourPicker() == Color.BLUE) {
			if (alliance == "Blue" ){
				jewel_Flick.setPosition(Flickr_Fwd);
			} else {
				jewel_Flick.setPosition(Flickr_Rev);
			}
			telemetry.addData("Jewel is ", "Blue !");
			telemetry.update();
		} else {
			if (alliance == "Blue" ){
				jewel_Flick.setPosition(Flickr_Rev);
			} else {
				jewel_Flick.setPosition(Flickr_Fwd);
			}
			telemetry.addData("Jewel is ", "Red !");
			telemetry.update();
		}
		sleep(1000); // Allow a bit of time for the colour sensor
	}
	
private double getAngle()
	{
		// We experimentally determined the Z axis is the axis we want to use for heading angle.
		// We have to process the angle because the imu works in euler angles so the Z axis is
		// returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
		// 180 degrees. We detect this transition and track the total cumulative angle of rotation.

		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

		double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

		if (deltaAngle < -180)
			deltaAngle += 360;
		else if (deltaAngle > 180)
			deltaAngle -= 360;

		globalAngle += deltaAngle;

		lastAngles = angles;

		if (globalAngle <=- 360)
			globalAngle = globalAngle + 360;
		else if (globalAngle >= 360)
			globalAngle = globalAngle - 360;

		return globalAngle;
	}	

}
