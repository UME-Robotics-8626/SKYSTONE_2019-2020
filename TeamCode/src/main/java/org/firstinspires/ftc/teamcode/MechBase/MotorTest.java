package org.firstinspires.ftc.teamcode.MechBase;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * The OnBotJava version of the MotorTest
 */
@TeleOp(name = "MotorTest", group = "TeleOP")
public class MotorTest extends AMyMecBase {

	/**
	 * This function is executed when this Op Mode is selected from the Driver Station.
	 */
	@Override
	public void runOpMode() throws InterruptedException {
		// initialize the control constants
		initConstants();
		// initialize the hardware before the opmode starts
		preStartInitialization();
		waitForStart();
		// the OpMode has started, do any initialization that needs to be postponed until the OpMode starts.
		postStartInitialization();
		resetDriveEncoders();
		// OK, the robot is ready to go
		// This is the main run loop,we use the dpad positions to run each of the motors forward for testing
		while (opModeIsActive()) {
			if (gamepad1.x) {
				break;
			}
			double leftFrontMotorPower = gamepad1.dpad_up ? 1.0 : 0.0;
			double leftBackMotorPower = gamepad1.dpad_right ? 1.0 : 0.0;
			double rightFrontMotorPower = gamepad1.dpad_down ? 1.0 : 0.0;
			double rightBackMotorPower = gamepad1.dpad_left ? 1.0 : 0.0;
			setPower(leftFrontMotorPower, leftBackMotorPower, rightFrontMotorPower, rightBackMotorPower);
			telemetry.addData("leftFrontPower", leftFrontMotorPower);
			telemetry.addData("leftFrontDistance", leftFrontMotor.getCurrentPosition());
			telemetry.addData("leftBackPower", leftBackMotorPower);
			telemetry.addData("leftBackDistance", leftBackMotor.getCurrentPosition());
			telemetry.addData("rightFrontPower", rightFrontMotorPower);
			telemetry.addData("rightFrontDistance", rightFrontMotor.getCurrentPosition());
			telemetry.addData("rightBackPower", rightBackMotorPower);
			telemetry.addData("rightBackDistance", rightBackMotor.getCurrentPosition());
			telemetry.update();
		}
	}
}