package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Autonomous(name="AutonomousStrafeLeftBridge", group="Linear OpMode")
//@Disabled
public class AutonomousStrafeLeftBridge extends LinearOpMode {

	/* Declare OpMode members. */

	Hardware robot   = new Hardware();
	private ElapsedTime     runtime = new ElapsedTime();

	static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
	static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
	static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
	static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
	static final double     DRIVE_SPEED = 1;
	static final double TURN_SPEED = .5;

	@Override
	public void runOpMode() throws InterruptedException {

		robot.init(hardwareMap);

		/*
		 * Initialize the drive system variables.
		 * The init() method of the hardware class does all the work here
		 */

		// Send telemetry message to signify robot waiting;
		telemetry.addData("Status", "Resetting Encoders");
		telemetry.update();

		robot.leftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.rightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		robot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		waitForStart();

		telemetry.addData("Thing are thing", "Thing");
		telemetry.update();

		//Strafes Right
		encoderStrafe(DRIVE_SPEED, 24, 24, 1.830);

		//Drives Forward
		encoderDrive(DRIVE_SPEED, 12, 12, 2);

		//Stops all motors
		stopAllMotors();
		sleep(1);

	}

	public void stopAllMotors(){
		robot.leftFrontWheel.setPower(0);
		robot.rightFrontWheel.setPower(0);
		robot.leftBackWheel.setPower(0);
		robot.rightBackWheel.setPower(0);
		robot.slideL.setPower(0);
		robot.slideR.setPower(0);
	}

	public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {

		//Defines the left and right wheel's directions
		int newLeftFrontTarget;
		int newLeftBackTarget;
		int newRightFrontTarget;
		int newRightBackTarget;

		// Ensure that the opmode is still active
		if (opModeIsActive()) {

			// Determine new target position, and pass to motor controller
			newLeftFrontTarget = robot.leftFrontWheel.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
			newLeftBackTarget = robot.leftBackWheel.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
			newRightFrontTarget = robot.rightFrontWheel.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
			newRightBackTarget = robot.rightBackWheel.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
			robot.leftFrontWheel.setTargetPosition(newLeftFrontTarget);
			robot.leftBackWheel.setTargetPosition(newLeftBackTarget);
			robot.rightFrontWheel.setTargetPosition(newRightFrontTarget);
			robot.rightBackWheel.setTargetPosition(newRightBackTarget);

			// Turn On RUN_TO_POSITION
			robot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			robot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			robot.leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			robot.rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			// reset the timeout time and start motion.
			runtime.reset();
			robot.leftFrontWheel.setPower(Math.abs(speed));
			robot.rightFrontWheel.setPower(Math.abs(speed));
			robot.leftBackWheel.setPower(Math.abs(speed));
			robot.rightBackWheel.setPower(Math.abs(speed));

			while (opModeIsActive() &&
					(runtime.seconds() < timeoutS) &&
					(robot.leftFrontWheel.isBusy() && robot.rightFrontWheel.isBusy() && robot.rightBackWheel.isBusy() && robot.rightBackWheel.isBusy())) {

				// Display it for the driver.
				telemetry.addData("Path1",  "Running to %7d :%7d", newLeftBackTarget, newLeftBackTarget,  newRightFrontTarget, newRightBackTarget);
				telemetry.addData("Path2",  "Running at %7d :%7d",
						robot.leftFrontWheel.getCurrentPosition(),
						robot.rightFrontWheel.getCurrentPosition(),
						robot.leftBackWheel.getCurrentPosition(),
						robot.rightBackWheel.getCurrentPosition());
				telemetry.update();
			}

			// Stop all motion;
			stopAllMotors();

			// Turn off RUN_TO_POSITION
			robot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			robot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			robot.leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			robot.rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

			//  sleep(250);   // optional pause after each move
		}
	}

	public void encoderStrafe(double speed, double leftInches, double rightInches, double timeoutS) {

		//Positive = leftInches  Negative = rightInches
		//Defines the left and right wheel's directions
		int newLeftFrontTarget;
		int newLeftBackTarget;
		int newRightFrontTarget;
		int newRightBackTarget;

		// Ensure that the opmode is still active
		if (opModeIsActive()) {

			// Determine new target position, and pass to motor controller
			newLeftFrontTarget = robot.leftFrontWheel.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
			newLeftBackTarget = robot.leftBackWheel.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
			newRightFrontTarget = robot.rightFrontWheel.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
			newRightBackTarget = robot.rightBackWheel.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
			robot.leftFrontWheel.setTargetPosition(newLeftFrontTarget);
			robot.leftBackWheel.setTargetPosition(newLeftBackTarget);
			robot.rightFrontWheel.setTargetPosition(newRightFrontTarget);
			robot.rightBackWheel.setTargetPosition(newRightBackTarget);

			// Turn On RUN_TO_POSITION
			robot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			robot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			robot.leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			robot.rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			// reset the timeout time and start motion.
			runtime.reset();
			robot.leftFrontWheel.setPower(Math.abs(speed));
			robot.rightFrontWheel.setPower(-1*Math.abs(speed));
			robot.leftBackWheel.setPower(-1*Math.abs(speed));
			robot.rightBackWheel.setPower(Math.abs(speed));

			while (opModeIsActive() &&
					(runtime.seconds() < timeoutS) &&
					(robot.leftFrontWheel.isBusy() && robot.rightFrontWheel.isBusy() && robot.rightBackWheel.isBusy() && robot.rightBackWheel.isBusy())) {

				// Display it for the driver.
				telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
				telemetry.addData("Path2", "Running at %7d :%7d",
						robot.leftFrontWheel.getCurrentPosition(),
						robot.rightFrontWheel.getCurrentPosition(),
						robot.leftBackWheel.getCurrentPosition(),
						robot.rightBackWheel.getCurrentPosition());
				telemetry.update();
			}

			// Stop all motion;
			stopAllMotors();

			// Turn off RUN_TO_POSITION
			robot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			robot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			robot.leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			robot.rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}

		//  sleep(250);   // optional pause after each move
	}

	public void encoderTurn(double speed, double leftInches, double rightInches, double timeoutS) {

		//Positive = leftInches  Negative = rightInches
		//Defines the left and right wheel's directions
		int newLeftFrontTarget;
		int newLeftBackTarget;
		int newRightFrontTarget;
		int newRightBackTarget;

		// Ensure that the opmode is still active
		if (opModeIsActive()) {

			// Determine new target position, and pass to motor controller
			newLeftFrontTarget = robot.leftFrontWheel.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
			newLeftBackTarget = robot.leftBackWheel.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
			newRightFrontTarget = robot.rightFrontWheel.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
			newRightBackTarget = robot.rightBackWheel.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
			robot.leftFrontWheel.setTargetPosition(newLeftFrontTarget);
			robot.leftBackWheel.setTargetPosition(newLeftBackTarget);
			robot.rightFrontWheel.setTargetPosition(newRightFrontTarget);
			robot.rightBackWheel.setTargetPosition(newRightBackTarget);

			// Turn On RUN_TO_POSITION
			robot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			robot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			robot.leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			robot.rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			// reset the timeout time and start motion.
			runtime.reset();
			robot.leftFrontWheel.setPower(Math.abs(speed));
			robot.rightFrontWheel.setPower(Math.abs(speed));
			robot.leftBackWheel.setPower(Math.abs(speed));
			robot.rightBackWheel.setPower(Math.abs(speed));

			while (opModeIsActive() &&
					(runtime.seconds() < timeoutS) &&
					(robot.leftFrontWheel.isBusy() && robot.rightFrontWheel.isBusy()&& robot.leftBackWheel.isBusy() && robot.rightBackWheel.isBusy())) {

				// Display it for the driver.
				telemetry.addData("Path1",  "Running to %7d :%7d",  newLeftBackTarget, newLeftBackTarget,  newRightFrontTarget, newRightBackTarget);
				telemetry.addData("Path2",  "Running at %7d :%7d",
						robot.leftFrontWheel.getCurrentPosition(),
						robot.rightFrontWheel.getCurrentPosition(),
						robot.leftFrontWheel.getCurrentPosition(),
						robot.rightFrontWheel.getCurrentPosition());
				telemetry.update();
			}

			// Stop all motion;
			stopAllMotors();

			// Turn off RUN_TO_POSITION
			robot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			robot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			robot.leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			robot.rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

			// optional pause after each move
			//sleep(time);
		}
	}

	public void encoderStrafeDL(double speed, double leftInches, double rightInches, double timeoutS) {

		encoderDrive(speed, leftInches, rightInches, timeoutS);

		encoderStrafe(speed, leftInches, rightInches, timeoutS);
	}

	public void encoderStrafeTurn(double speed, double leftInches, double rightInches, double timeoutS) {
		//Positive = leftInches  Negative = rightInches
		//Defines the left and right wheel's directions
		int newLeftFrontTarget;
		int newLeftBackTarget;
		int newRightFrontTarget;
		int newRightBackTarget;

		// Ensure that the opmode is still active
		if (opModeIsActive()) {

			// Determine new target position, and pass to motor controller
			newLeftFrontTarget = robot.leftFrontWheel.getCurrentPosition() / (int) (leftInches * COUNTS_PER_INCH);
			newLeftBackTarget = robot.leftBackWheel.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
			newRightFrontTarget = robot.rightFrontWheel.getCurrentPosition() / (int) (rightInches * COUNTS_PER_INCH);
			newRightBackTarget = robot.rightBackWheel.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
			robot.leftFrontWheel.setTargetPosition(newLeftFrontTarget);
			robot.leftBackWheel.setTargetPosition(newLeftBackTarget);
			robot.rightFrontWheel.setTargetPosition(newRightFrontTarget);
			robot.rightBackWheel.setTargetPosition(newRightBackTarget);

			// Turn On RUN_TO_POSITION
			robot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			robot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			robot.leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			robot.rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			// reset the timeout time and start motion.
			runtime.reset();
			robot.leftFrontWheel.setPower(-1*Math.abs(speed));
			robot.rightFrontWheel.setPower(Math.abs(speed));
			robot.leftBackWheel.setPower(Math.abs(speed));
			robot.rightBackWheel.setPower(-1*Math.abs(speed));

			while (opModeIsActive() &&
					(runtime.seconds() < timeoutS) &&
					(robot.leftFrontWheel.isBusy() && robot.rightFrontWheel.isBusy() && robot.rightBackWheel.isBusy() && robot.rightBackWheel.isBusy())) {

				// Display it for the driver.
				telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
				telemetry.addData("Path2", "Running at %7d :%7d",
						robot.leftFrontWheel.getCurrentPosition(),
						robot.rightFrontWheel.getCurrentPosition(),
						robot.leftBackWheel.getCurrentPosition(),
						robot.rightBackWheel.getCurrentPosition());
				telemetry.update();
			}

			// Stop all motion;
			stopAllMotors();

			// Turn off RUN_TO_POSITION
			robot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			robot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			robot.leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			robot.rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}

		//  sleep(250);   // optional pause after each move
	}

}