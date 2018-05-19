package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;



/**
 * Created by Metal Minds on 7/04/2018.
 */
@TeleOp

public class Simon_IMU_test extends LinearOpMode {

	private DcMotor front_Left;
	private DcMotor front_Right;
	private DcMotor rear_Left;
	private DcMotor rear_Right;
	private BNO055IMU imu;
	private Orientation lastAngles = new Orientation();
	double globalAngle, power = .30, correction;

	@Override
	public void runOpMode() throws InterruptedException {

		front_Left = hardwareMap.get(DcMotor.class, "Front_Left");
		front_Right = hardwareMap.get(DcMotor.class, "Front_Right");
		rear_Left = hardwareMap.get(DcMotor.class, "Rear_Left");
		rear_Right = hardwareMap.get(DcMotor.class, "Rear_Right");

//		front_Left.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//		front_Right.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//		rear_Left.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//		rear_Right.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

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

		waitForStart();
		while (opModeIsActive() && !isStopRequested()) {

			float gamepad1LeftY = -gamepad1.left_stick_y;
			float gamepad1LeftX = gamepad1.left_stick_x;
			float gamepad1RightX = gamepad1.right_stick_x;

			// holonomic formulas

			float FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
			float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
			float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
			float BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

			// clip the right/left values so that the values never exceed +/- 1
			FrontRight = Range.clip(FrontRight, -1, 1);
			FrontLeft = Range.clip(FrontLeft, -1, 1);
			BackLeft = Range.clip(BackLeft, -1, 1);
			BackRight = Range.clip(BackRight, -1, 1);

			// write the values to the motors
			front_Right.setPower(FrontRight);
			front_Left.setPower(FrontLeft);
			rear_Left.setPower(BackLeft);
			rear_Right.setPower(BackRight);


			telemetry.addData("ZAxis", getAngle());
			telemetry.update();

		}
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
