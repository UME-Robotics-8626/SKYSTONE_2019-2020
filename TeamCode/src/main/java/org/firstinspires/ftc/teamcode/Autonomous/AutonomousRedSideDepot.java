package org.firstinspires.ftc.teamcode.Autonomous;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

import java.util.List;

@Autonomous(name="AutonomousBlueSideDepot", group="Linear OpMode")
//@Disabled
public class AutonomousRedSideDepot extends LinearOpMode {

	/* Declare OpMode members. */

	Hardware robot   = new Hardware();
	private ElapsedTime     runtime = new ElapsedTime();

	static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
	static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
	static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
	static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
	static final double     DRIVE_SPEED = 1;
	static final double TURN_SPEED = .5;
	private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
	private static final String LABEL_FIRST_ELEMENT = "Stone";
	private static final String LABEL_SECOND_ELEMENT = "skystone";

	private static final String VUFORIA_KEY = "AWPg8xD/////AAABmfbsugEr1EsQjZhOZDkPpUEi09sxQgeOizNeQwf9a/M+l1ZKXrfujjzHy0CgbT59xZurtuCY4QihV8G7MgqpNs/IkgY8HEm/mN/LCmxsCRfFV0uk7rOos/2uwL2vDwEf2ytqI/cxXiBypjxippnzpqp28H6K5Vf6rJq/mDgonb0pMsXlbCJXZtvFYFvBCtJsaH/ufpDOaY8mzSifVzYxjF06wQPz9HJ93I10htWgPu8SdgKrSWzZruY8Tp+kaMM5SJ/tEIn0VqeTcB48xCTzM9Oev9hvH5kaEFrizZBro6T/HfHWKuK/MrOffSj46XxGecl/am9zMbcbPRztXwmI8Ei8WPWQIN5YJAPn3b6SvbnH";


	private VuforiaLocalizer vuforia;
	private TFObjectDetector tfod;
	NormalizedColorSensor colorSensor;
	View relativeLayout;
	DistanceSensor sensorRange;
	Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

	int ifCount = 0;
	//Color is for detecting colors, 0 = none, 1 = red, 2 = blue
	int color = 0;
	int distanceComplete = 0;

	@Override
	public void runOpMode() throws InterruptedException {

		sensorRange = hardwareMap.get(DistanceSensor.class, "DistanceSensor");

		int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
		relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

		try {
			runSample(); // actually execute the sample
		} finally {

			relativeLayout.post(new Runnable() {
				public void run() {
					relativeLayout.setBackgroundColor(Color.WHITE);
				}
			});
		}

		/** Wait for the game to begin */
		telemetry.addData(">", "Press Play to start op mode");
		telemetry.update();

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

		//Sets the Grabber's init position
		robot.claw.setPosition(0);
		robot.clawH.setPosition(0);
		robot.clawV.setPosition(0);

		//Sets the foundation hook's init position
		robot.hook.setPosition(-1);

		//Sets the Claws init Power
		//robot.grabLeft.setPower(0);
		//robot.grabRight.setPower(0);

		waitForStart();

		telemetry.addData("Thing are thing", "thing");
		telemetry.update();

		//Move right
		//encoderStrafe(0.725, -34.,-34,1.830);
		encoderStrafe(1, 34., 34, 1.830);

		if (opModeIsActive()) {
			while (opModeIsActive() && ifCount < 2) {
				if (tfod != null) {
					// getUpdatedRecognitions() will return null if no new information is available since
					// the last time that call was made.
					List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
					if (updatedRecognitions != null) {
						telemetry.addData("# Object Detected", updatedRecognitions.size());

						// step through the list of recognitions and display boundary info.
						int i = 0;
						for (Recognition recognition : updatedRecognitions) {

							telemetry.addData("# Object Detected", updatedRecognitions.size());
							telemetry.addData(String.format(" label(%d)", i), recognition.getLabel());
							telemetry.addData(String.format(" left(%d)", i), "%.03f", recognition.getLeft());
							telemetry.addData(String.format(" right(%d)", i), "%.03f", recognition.getRight());
							telemetry.addData(String.format(" top(%d)", i), "%.03f", recognition.getTop());
							telemetry.addData(String.format(" bottom(%d)", i), "%.03f", recognition.getBottom());

							if (recognition.getLabel() != null || ifCount == 2) {

								telemetry.addData("me", "me");

								//Grabs brick
								robot.clawV.setPosition(1);
								robot.claw.setPosition(1);
								sleep(1000);

								stopAllMotors();
								sleep(1);

								//Strafe left
								encoderStrafe(DRIVE_SPEED, -1, -1, 1);

								//Init motors
								stopAllMotors();
								sleep(1);

								while(opModeIsActive() && distanceComplete == 0) {

									// generic SensorDistance methods.
									telemetry.addData("deviceName",sensorRange.getDeviceName() );
									telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));

									// Rev2mDistanceSensor specific methods.
									telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
									telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

									telemetry.update();

									robot.leftFrontWheel.setPower(1);
									robot.leftBackWheel.setPower(1);
									robot.rightFrontWheel.setPower(1);
									robot.rightBackWheel.setPower(1);

									if (sensorRange.getDistance(DistanceUnit.INCH) <= 30){

										telemetry.addData("Wall Sensed", "Wall");

										stopAllMotors();

										distanceComplete = 1;

									}

								}

								//Init motors
								stopAllMotors();
								sleep(1);

								//Raises claw
								robot.clawV.setPosition(-1);

								//This goes into the foundation
								encoderStrafe(DRIVE_SPEED, -5, -5, 1.2);

								//Init motors
								stopAllMotors();
								sleep(1);

								//Lowers claw and releases stone
								robot.clawV.setPosition(1);
								robot.claw.setPosition(-1);
								sleep(200);

								robot.clawV.setPosition(-1);
								sleep(100);

								robot.claw.setPosition(-1);

								//Strafe left to avoid collision
								encoderStrafe(DRIVE_SPEED, -2, -2, 1);

								//Init motors
								stopAllMotors();
								sleep(1);

								//Turn right 90 degrees
								encoderTurn(1, -56, -56, 1);

								//Init motors
								stopAllMotors();
								sleep(1);

								//Reverses into foundation
								encoderDrive(1, -4.5, -4.5, 1);

								//Init motors
								stopAllMotors();
								sleep(1);

								//Lowers hook to hull foundation
								robot.hook.setPosition(-1);

								//Moves forward
								encoderDrive(DRIVE_SPEED, 3, 3, 1);

								//Init motors
								stopAllMotors();
								sleep(1);

								//Lift claws
								robot.hook.setPosition(1);
								sleep(100);

								if (opModeIsActive()) {

									while (opModeIsActive() && color != 2) {

										robot.leftFrontWheel.setPower(-1);
										robot.leftBackWheel.setPower(1);
										robot.rightFrontWheel.setPower(1);
										robot.rightBackWheel.setPower(-1);

										runSample();

									}
								}

								//Init motors
								stopAllMotors();
								sleep(1);

							} else {

								telemetry.addData("Hi", "Hi");
								sleep(500);

								encoderDrive(DRIVE_SPEED, -6, -6, 1);

							}

							ifCount = ifCount + 1;

						}
						telemetry.update();
					}
				}
			}
		}

		if (tfod != null) {
			tfod.shutdown();
		}

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

	private void initVuforia() {
		/*
		 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
		 */
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		parameters.vuforiaLicenseKey = VUFORIA_KEY;

		//  Instantiate the Vuforia engine
		vuforia = ClassFactory.getInstance().createVuforia(parameters);

		// Loading trackables is not necessary for the TensorFlow Object Detection engine.
	}

	private void initTfod() {
		int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
				"tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
		tfodParameters.minimumConfidence = 0.8;
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
	}

	public void runSample () throws InterruptedException {

		// values is a reference to the hsvValues array.
		float[] hsvValues = new float[3];
		final float values[] = hsvValues;

		// bPrevState and bCurrState keep track of the previous and current state of the button
		boolean bPrevState = false;
		boolean bCurrState = false;

		initVuforia();

		if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
			initTfod();
		} else {
			telemetry.addData("Sorry!", "This device is not compatible with TFOD");
		}

		if (tfod != null) {
			tfod.activate();
		}

		// Check the status of the x button on the gamepad
		bCurrState = gamepad1.x;

		// If the button state is different than what it was, then act
		if (bCurrState != bPrevState) {
			// If the button is (now) down, then toggle the light
			if (bCurrState) {
				if (colorSensor instanceof SwitchableLight) {
					SwitchableLight light = (SwitchableLight)colorSensor;
					light.enableLight(!light.isLightOn());
				}
			}
		}
		bPrevState = bCurrState;

		// Read the sensor
		NormalizedRGBA colors = colorSensor.getNormalizedColors();

		/** Use telemetry to display feedback on the driver station. We show the conversion
		 * of the colors to hue, saturation and value, and display the the normalized values
		 * as returned from the sensor.
		 * @see <a href="http://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html">HSV</a>*/

		Color.colorToHSV(colors.toColor(), hsvValues);
		telemetry.addLine()
				.addData("H", "%.3f", hsvValues[0])
				.addData("S", "%.3f", hsvValues[1])
				.addData("V", "%.3f", hsvValues[2]);
		telemetry.addLine()
				.addData("a", "%.3f", colors.alpha)
				.addData("r", "%.3f", colors.red)
				.addData("g", "%.3f", colors.green)
				.addData("b", "%.3f", colors.blue);

		/** We also display a conversion of the colors to an equivalent Android color integer.
		 * @see Color */
		int color = colors.toColor();
		telemetry.addLine("raw Android color: ")
				.addData("a", "%02x", Color.alpha(color))
				.addData("r", "%02x", Color.red(color))
				.addData("g", "%02x", Color.green(color))
				.addData("b", "%02x", Color.blue(color));

		if (colors.red >= 100) {

			telemetry.addData("me", "me");
			color = 1;

		} else if (colors.blue >= 100) {

			telemetry.update();
			color = 2;

		} else {

			telemetry.update();

		}

		float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
		colors.red   /= max;
		colors.green /= max;
		colors.blue  /= max;
		color = colors.toColor();

		telemetry.addLine("normalized color:  ")
				.addData("a", "%02x", Color.alpha(color))
				.addData("r", "%02x", Color.red(color))
				.addData("g", "%02x", Color.green(color))
				.addData("b", "%02x", Color.blue(color));
		telemetry.update();

		// convert the RGB values to HSV values.
		Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);

		// change the background color to match the color detected by the RGB sensor.
		// pass a reference to the hue, saturation, and value array as an argument
		// to the HSVToColor method.
		relativeLayout.post(new Runnable() {
			public void run() {
				relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
			}
		});
	}
}