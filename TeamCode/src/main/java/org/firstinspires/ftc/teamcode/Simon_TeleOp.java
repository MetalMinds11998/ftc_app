/*
Copyright 2017 FIRST Tech Challenge Team 11998

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp

public class Simon_TeleOp extends LinearOpMode {
//	private Gyroscope imu;
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

	private ElapsedTime	 runtime = new ElapsedTime();


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
		
		// front_Left.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
		// front_Right.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
		// rear_Left.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
		// rear_Right.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
		
		// declare variables for the left and right motor powers



		float slide_power;
		float arm_power;
		//declare variables for gripper control
		double Gripper1_Min = 0.0;
		double Gripper1_Max = 0.8; //max closed position
		double Gripper1_Init = 0.35; // Start point of gripper 1
		double Gripper2_Min = 0.05; //max closed position
		double Gripper2_Max = 0.9;
		double Gripper2_Init = 0.45; // Start point of gripper 2
		double Gripper3_Min = -0.9;
		double Gripper3_Max = 0.98;
		double Gripper_Inc = 0.004;
		double grip_1Now;
		double grip_2Now;
		double scratch = 0;
		double colourArm_Max = 0.64;
		double Flickr_Init = 0.5; //  Start point for the jewel flick servo

		gripper_1.setPosition(Gripper1_Init);
		gripper_2.setPosition(Gripper2_Init);
		gripper_3.setPosition(Gripper3_Min);
		double currPos;
		colour_arm.setPosition(0.64);
		jewel_Flick.setPosition(Flickr_Init);
		
		telemetry.addData("Status", "Initialized");
		telemetry.update();
		// Wait for the game to start (driver presses PLAY)
		waitForStart();



		// run until the end of the match (driver presses STOP)
		while (opModeIsActive()) {
			
			float gamepad1LeftY = (-gamepad1.left_stick_y / 4) * 3;
			float gamepad1LeftX = gamepad1.left_stick_x / 2;
			float gamepad1RightX = gamepad1.right_stick_x / 4;

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

//			// front lift/lower motor if right bumber pressed raise, if right trigger pressed lower
			if (gamepad2.right_bumper) {
				telemetry.addData("right_bumper", "On" );

				front_Lift.setPower(0.5);
			} else {
				if (gamepad2.right_trigger > 0.0) {
					front_Lift.setPower(-0.5);
				} else {
					front_Lift.setPower(0.0);
				}
			}

			// rear lift/lower motor if left bumber pressed raise, if right trigger pressed lower
			runtime.reset();
			
			if (gamepad2.left_bumper && runtime.seconds() < 2) {
				
				rear_Lift.setPower(0.5);
			} else {
				if (gamepad2.left_trigger > 0 && runtime.seconds() < 2) {
					rear_Lift.setPower(-0.5);
				} else {
					rear_Lift.setPower(0.0);
				}
			}
  
  
			// Glyph and Relic Slide Extend and Retract
			slide_power = gamepad2.right_stick_y;
			relic_Slide.setPower(-slide_power/3);
			
		
			
			// Glyph gripper
			// Servo control
			if (gamepad2.x){ // Close Gripper
				if (gripper_3.getPosition() < Gripper3_Max) {
					gripper_3.setPosition(gripper_3.getPosition() + Gripper_Inc);
				}
			} else if (gamepad2.b) { //Open Gripper
				if (gripper_3.getPosition() > Gripper3_Min) {
					gripper_3.setPosition(gripper_3.getPosition() - Gripper_Inc);
				}
			}

			//Glyph Control incremental open/close
		if (gamepad2.dpad_right){
				if (gripper_1.getPosition() < Gripper1_Max) {
					gripper_1.setPosition(gripper_1.getPosition() + Gripper_Inc);
				}
				if (gripper_2.getPosition() > Gripper2_Min) {
					gripper_2.setPosition(gripper_2.getPosition() - Gripper_Inc);
				}
			}
			if (gamepad2.dpad_left){
				if (gripper_1.getPosition() > Gripper1_Min) {
					gripper_1.setPosition(gripper_1.getPosition() - Gripper_Inc);
				}
				if (gripper_2.getPosition() < Gripper2_Max) {
					gripper_2.setPosition(gripper_2.getPosition() + Gripper_Inc);
				}

			}


			currPos = colour_arm.getPosition();
			if (gamepad2.right_bumper) {
			if (currPos < colourArm_Max) {
				colour_arm.setPosition(currPos + 0.02);
				sleep(100);
			}
			}
			
  
//			telemetry.addData("Left Power : ", left_power);
//			telemetry.addData("Right Power : ", right_power);
			telemetry.addData("Gripper_1 : ", gripper_1.getPosition());
			telemetry.addData("Gripper_2 :", gripper_2.getPosition());
			telemetry.update();
		}
	}
}
