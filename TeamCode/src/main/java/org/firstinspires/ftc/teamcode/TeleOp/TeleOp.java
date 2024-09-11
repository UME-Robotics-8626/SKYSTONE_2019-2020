package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;

/*
ADB guide can be found at:
https://ftcprogramming.wordpress.com/2015/11/30/building-ftc_app-wirelessly/

This code is written as an example only.
Obviously, it was not tested on your team's robot.
Teams who use and reference this code are expected to understand code they use.

If you use our code and see us at competition, come say hello!
*/

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {

	//Hardware robot = new Hardware();

	//The horizontal linear slide system's motors are set to null
	private static DcMotor slideL = null; //Linear slide left motor
	private static DcMotor slideR = null; //Linear slide right motor

	//The vertical linear slide system's motors are set to null
	private static CRServo slideH = null; //Linear slide horizontal

	//The intake system's motors are set to null
	private static DcMotor intakeLM = null; //Intake left motor
	private static DcMotor intakeRM = null; //Intake right motor

	//The intake system's continuous servos are set to null
	private static CRServo intakeLS = null; //Intake right servo
	private static CRServo intakeRS = null; //Intake left servo

	//The claw rotate system's servos are set to null
	private static Servo clawH = null; //Claw rotates the claw horizontally
	private static Servo clawV = null; //Claw rotates the claw vertically

	//The claw grab system's servos are set to null
	private static Servo claw = null; //Claw opens and closes

	//The foundation hook system's servos are set to null
	private static Servo hook = null; //Foundation hook

	//The capstone system's servos are set to null
	private static Servo capstone = null; //Capstone dropper

	//The stone clamp system's servos are set to null
	private static Servo clampH = null; //Stone clamp horizontal
	private static Servo clampV = null; //Stone clamp vertical

	private static DcMotor leftFrontMotor = null; //Left front motor
	private static DcMotor leftBackMotor = null; //Left back motor
	private static DcMotor rightFrontMotor = null; //Right front motor
	private static DcMotor rightBackMotor = null; //Right back motor


	double ticsPerInchForward;
	double ticsPerInchSideways;
	double mtrAccelMin;
	double mtrAccelTics;
	double mtrAccelDegs;
	double mtrDecelMin;
	double mtrDecelTics;
	double mtrDecelDegs;

	double stickDeadband;
	double stickSensitivity;
	// The gamepad1 stick values conditioned for deadband and sensitivity
	double rightStickX;
	double rightStickY;
	double leftStickX;
	double leftStickY;

	double kp;

	double heading;
	int headingRevs;
	double headingRawLast;
	double expectedHeading;
	boolean inDrive;
	boolean inTurn;
	boolean inStrafe;


	// The IMU sensor object
	private static BNO055IMU IMU = null; // The IMU, generally on the hub controlling the motors

	//Sets the drive and strafe counts to 0
	int driveCount = 0;
	int strafeCount = 0;

	//Capstone Count is for the capstone
	int capstoneCount = 0;

	//Sets the horizontal claw and vertical claw counts to 0
	int clawVCount = 0;
	int clawCount = 0;

	//Sets the hook count to
	int clampCount0 = 0;
	int clampCount90 = 0;
	int clampCount180 = 0;
	int clampCountV = 0;

	//Sets the hook count to 0
	int hookCount = 0;

	int startCount = 0;

	int mecamumDriveCount = 0;

	int opModeIsActiveCount = 0;

	protected void initConstants() {

		// These will be specific to your robot base, and the gear ratio you selected when you built your robot. Run the
		// calibration and tune these values for your robot.
		ticsPerInchForward = 295;
		ticsPerInchSideways = 350;

		// These values were selected to be reasonable for the TileRunner base with a 1:1 gear ratio. Refer to the README notes
		// and tune these for your robot. Refer to the project readme for notes about the power ramp function being
		// controlled by these variables.
		mtrAccelMin = 0.3;
		mtrAccelTics = 1000;
		mtrAccelDegs = 20;

		mtrDecelMin = 0.2;
		mtrDecelTics = 3000;
		mtrDecelDegs = 30;

		// These are for conditioning stick values from the logitech gamepad to make the robot more controllable
		// for the driver.
		stickDeadband = 0.05;
		stickSensitivity = 2.0;

		// This is the proportional multiplier for the IMU PID loop (we only use P), that tries to maintain the robot heading to
		// the expected heading. NOTE: IMU
		kp = 0.05;
	}

	protected void preStartInitialization() {

		// Initialize the hardware variables from the hardware map
		leftFrontMotor = hardwareMap.dcMotor.get("LeftFrontMotor");
		leftBackMotor = hardwareMap.dcMotor.get("LeftBackMotor");
		rightFrontMotor = hardwareMap.dcMotor.get("RightFrontMotor");
		rightBackMotor = hardwareMap.dcMotor.get("RightBackMotor");

		IMU = hardwareMap.get(BNO055IMU.class, "Imu");

		// Initialize the motors for the correct directions and braking
		leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		// This initializes the IMU for use of the gyros. NOTE: gyros precess, and the REV hub may not be perfectly
		// aligned with the frame. Make no assumptions about the heading reported from the IMU. Read the IMU heading when
		// the OpMode starts, and use that as the reference.
		BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
		imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		imuParams.calibrationDataFile = "BNO055IMUCalibration.json";
		IMU.initialize(imuParams);

		while (opModeIsActiveCount == 1) {
			if (IMU.isGyroCalibrated()) {
				break;
			}
		}
	}

	protected void postStartInitialization() {
		// setup heading calculation and save the current heading as the expected heading, i.e. the current heading becomes the
		// reference expected heading fo all actions after this.
		headingRevs = 0;
		headingRawLast = 0.0;
		recomputeHeading();
		expectedHeading = heading;
		inDrive = false;
		inTurn = false;
		inStrafe = false;
	}

	void resetDriveEncoders() {
		leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	protected void setPower(final double rightFrontPower, final double rightBackPower,final double leftFrontPower,final double leftBackPower) {
		rightFrontMotor.setPower(rightFrontPower);
		rightBackMotor.setPower(rightBackPower);
		leftFrontMotor.setPower(leftFrontPower);
		leftBackMotor.setPower(leftBackPower);
	}

	protected void setDrive(double forward, double sideways, double rotation) {

		double scale = 1.0;
		double max = Math.abs(forward) + Math.abs(sideways) + Math.abs(rotation);
		if (max > 1.0) {
			scale = 1.0 / max;
		}
		double leftFrontPower = scale * ((forward - sideways) + rotation);
		double leftBackPower = scale * ((forward + sideways) + rotation);
		double rightFrontPower = scale * ((-forward + sideways) + rotation);
		double rightBackPower = scale * ((-forward - sideways) + rotation);
		setPower(rightFrontPower, rightBackPower, leftFrontPower, leftBackPower);

	}

	protected void setStrafe(double forward, double sideways, double rotation) {
		double scale = 1.0;
		double max = Math.abs(forward) + Math.abs(sideways) + Math.abs(rotation);
		if (max > 1.0) {
			scale = 1.0 / max;
		}
		double leftFrontPower = scale * (forward - sideways - rotation);
		double leftBackPower = scale * ((forward + sideways) - rotation);
		double rightFrontPower = scale * ((forward + sideways) + rotation);
		double rightBackPower = scale * ((forward - sideways) + rotation);
		setPower(-rightFrontPower, -rightBackPower, leftFrontPower, leftBackPower);
	}

	protected void setTurn(double forward, double sideways, double rotation) {
		double scale = 1.0;
		double max = Math.abs(forward) + Math.abs(sideways) + Math.abs(rotation);
		if (max > 1.0) {
			scale = 1.0 / max;
		}
		double leftFrontPower = scale * (-forward - sideways - rotation);
		double leftBackPower = scale * ((forward - sideways) + rotation);
		double rightFrontPower = scale * ((forward + sideways) + rotation);
		double rightBackPower = scale * ((-forward + sideways) - rotation);
		setPower(-rightFrontPower, -rightBackPower, leftFrontPower, leftBackPower);
	}

	protected void conditionSticks() {
		rightStickX = conditionStickValue(gamepad1.right_stick_x);
		rightStickY = conditionStickValue(gamepad1.right_stick_y);
		leftStickX = conditionStickValue(gamepad1.left_stick_x);
		leftStickY = conditionStickValue(-gamepad1.left_stick_y);
	}

	private double conditionStickValue(double stickValue) {
		return conditionStickValue(stickValue, stickDeadband, stickSensitivity);
	}

	private double conditionStickValue(double stickValue, double deadband, double sensitivity) {
		if (Math.abs(stickValue) < deadband)  {
			return 0.0;
		}
		double stickSign = (stickValue < 0.0) ? -1.0 : 1.0;
		double deadbandCorrected = (Math.abs(stickValue) - deadband) / (1.0 - deadband);
		return stickSign * Math.pow(deadbandCorrected, sensitivity);
	}

	protected void recomputeHeading() {
		Orientation angles = IMU.getAngularOrientation();
		// This assumes the base of the REV hub is parallel to the ground plane.
		float heading_raw = angles.firstAngle;
		// This is the logic for detecting and correcting for the IMU discontinuity at +180degrees and -180degrees.
		if (headingRawLast < -150.0 && heading_raw > 0.0) {
			// The previous raw IMU heading was negative and close to the discontinuity, and it is now positive. We have
			// gone through the discontinuity so we decrement the heading revolutions by 1 (we completed a negative
			// revolution). NOTE: the initial check protects from the case that the heading is near 0 and goes continuously
			// through 0, which is not the completion of a revolution.
			headingRevs--;
		} else if (headingRawLast > 150.0 && heading_raw < 0.0) {
			// The previous raw IMU heading was positive and close to the discontinuity, and it is now negative. We have
			// gone through the discontinuity so we increment the heading revolutions by 1 (we completed a positive
			// revolution). NOTE: the initial check protects from the case that the heading is near 0 and goes continuously
			// through 0, which is not the completion of a revolution.
			headingRevs++;
		}
		heading = -((headingRevs * 360.0) + heading_raw);
		headingRawLast = heading_raw;
	}

	private double powerAccelDecel(double currentTics, double targetTics, double maxPower,
	                               double accelMin, double accelTics, double decelMin, double decelTics) {
		if (currentTics <= 0) {
			return accelMin;
		}
		if (currentTics >= targetTics) {
			return 0.0;
		}
		double mtrPower = maxPower;
		if (currentTics < accelTics) {
			double accelPower = accelMin + ((1 - accelMin) * (currentTics / accelTics));
			if (accelPower < mtrPower) {
				mtrPower = accelPower;
			}
		}
		if (currentTics > targetTics - decelTics) {
			double decelPower = decelMin + ((1 - decelMin) * ((targetTics - currentTics) / decelTics));
			if (decelPower < mtrPower) {
				mtrPower = decelPower;
			}
		}
		return mtrPower;
	}

	private double forwardTics() {
		// NOTE:, to go forward all 4 drive motors are going forward.
		return leftFrontMotor.getCurrentPosition() + leftBackMotor.getCurrentPosition() + rightFrontMotor.getCurrentPosition() + rightBackMotor.getCurrentPosition();
	}


	private double sidewaysTics() {
		// NOTE:, to go sideways, the front right and left rear drive motors are reversed.
		return (rightBackMotor.getCurrentPosition() + leftFrontMotor.getCurrentPosition()) - (rightFrontMotor.getCurrentPosition() + leftBackMotor.getCurrentPosition());
	}

	final protected void move(double inches, double degrees) {
		move(inches, degrees, 1.0);
	}

	final protected void move(double inches, double degrees, double maxPower) {
		move(inches, degrees, maxPower, mtrAccelMin, mtrAccelTics, mtrDecelMin, mtrDecelTics);
	}

	final protected void move(double inches, double degrees, double maxPower,
	                          double accelMin, double accelTics, double decelMin, double decelTics) {
		resetDriveEncoders();
		double sin = Math.sin(degrees / 180 * Math.PI);
		double cos = Math.cos(degrees / 180 * Math.PI);
		// get the forward and sideways components of the move. To give the greatest accuracy of the move we will use the one
		// that is the longest (greatest number of encoder tics) to decide when the move has completed.
		double forwardMaxSpeed = Math.abs(cos);
		double forwardInches = cos * inches;
		double forwardDirectionMult = (forwardInches > 0.0) ? 1.0 : -1.0;

		double sidewaysMaxSpeed = Math.abs(sin);
		double sidewaysInches = sin * inches;
		double sidewaysDirectionMult = (sidewaysInches > 0) ? 1.0 : -1.0;

		double target_tics = (forwardMaxSpeed >= sidewaysMaxSpeed) ?
				ticsPerInchForward * forwardInches * forwardDirectionMult :
				ticsPerInchSideways * sidewaysInches * sidewaysDirectionMult;
		while (opModeIsActiveCount == 1) {
			double current_tics = (forwardMaxSpeed >= sidewaysMaxSpeed) ?
					forwardTics() * forwardDirectionMult : sidewaysTics() * sidewaysDirectionMult;
			if (current_tics >= target_tics) {
				break;
			}
			double speed_mult = powerAccelDecel(current_tics, target_tics, maxPower, accelMin, accelTics, decelMin, decelTics);
			recomputeHeading();
			double error = expectedHeading - heading;
			setDrive(forwardMaxSpeed * speed_mult * forwardDirectionMult,
					sidewaysMaxSpeed * speed_mult * sidewaysDirectionMult,
					kp * error);

			setStrafe(forwardMaxSpeed * speed_mult * forwardDirectionMult,
					sidewaysMaxSpeed * speed_mult * sidewaysDirectionMult,
					kp * error);

			setTurn(forwardMaxSpeed * speed_mult * forwardDirectionMult,
					sidewaysMaxSpeed * speed_mult * sidewaysDirectionMult,
					kp * error);
		}
		setDrive(0.0, 0.0, 0.0);

		setStrafe(0.0, 0.0, 0.0);

		setTurn(0.0, 0.0, 0.0);
	}

	protected void rotate(double degrees) {
		rotate(degrees, 1.0);
	}

	protected void rotate(double degrees, double maxPower) {
		rotate(degrees, maxPower, mtrAccelMin, mtrAccelDegs, mtrDecelMin, mtrDecelDegs);
	}

	protected void rotate(double degrees, double maxPower,
	                      double accelMin, double accelDegs, double decelMin, double decelDegs) {
		expectedHeading = expectedHeading + degrees;
		recomputeHeading();
		// Rotate as specified
		turn(expectedHeading - heading, maxPower, accelMin, accelDegs, decelMin, decelDegs);
		// Test the heading and correct for error
		recomputeHeading();
		turn(expectedHeading - heading, maxPower, accelMin, accelDegs, decelMin, decelDegs);
	}

	private void turn(double degrees, double maxPower,
	                  double accelMin, double accelDegs, double decelMin, double decelDegs) {
		resetDriveEncoders();
		double direction_mult = (degrees > 0) ? 1.0 : -1.0;
		double start_heading = heading;
		while (opModeIsActiveCount == 1 && direction_mult * (heading - start_heading) < degrees * direction_mult) {
			setDrive(0.0, 0.0, powerAccelDecel(direction_mult * (heading - start_heading), degrees * direction_mult,
					maxPower, accelMin, accelDegs, decelMin, decelDegs) * direction_mult);
			recomputeHeading();
		}
		setDrive(0.0, 0.0, 0.0);
	}

	@Override
	public void init() {

		//Gets the vertical linear slide's system from hardware
		slideL = hardwareMap.dcMotor.get("SlideL"); //Linear slide left motor
		slideR = hardwareMap.dcMotor.get("SlideR"); //Linear slide right motor

		//Gets the horizontal linear slide's system from hardware
		slideH = hardwareMap.crservo.get("SlideH"); //Linear slide horizontal

		//Gets the intake system's motors from hardware
		intakeLM = hardwareMap.dcMotor.get("IntakeLM"); //Intake left motor
		intakeRM = hardwareMap.dcMotor.get("IntakeRM"); //Intake right motor

		//Gets the intake system's continuous servos from hardware
		intakeLS = hardwareMap.crservo.get("IntakeLS");
		intakeRS = hardwareMap.crservo.get("IntakeRS");

		//Gets the claws system's servos from hardware
		clawH = hardwareMap.servo.get("ClawH"); //Claw rotates the claw vertically
		clawV = hardwareMap.servo.get("ClawV"); //Claw rotates the claw vertically
		claw = hardwareMap.servo.get("Claw"); //Claw opens and closes

		//Gets the hook system's servos from hardware
		hook = hardwareMap.servo.get("Hook"); //Foundation hook

		capstone = hardwareMap.servo.get("Capstone"); //Capstone dropper

		//Gets the capstone system's servos from hardware
		clampH = hardwareMap.servo.get("ClampH"); //Stone clamp horizontal
		clampV = hardwareMap.servo.get("ClampV"); //Stone clamp vertical

		//Gets the drive system's motors from hardware
		leftFrontMotor = hardwareMap.dcMotor.get("LeftFrontMotor"); //Left front motor
		leftBackMotor = hardwareMap.dcMotor.get("LeftBackMotor"); //Left back motor
		rightFrontMotor = hardwareMap.dcMotor.get("RightFrontMotor"); //Right front motor
		rightBackMotor = hardwareMap.dcMotor.get("RightBackMotor"); //Right back motor

		IMU = hardwareMap.get(BNO055IMU.class, "Imu");

		// Initialize the motors for the correct directions and braking
		leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		// This initializes the IMU for use of the gyros. NOTE: gyros precess, and the REV hub may not be perfectly
		// aligned with the frame. Make no assumptions about the heading reported from the IMU. Read the IMU heading when
		// the OpMode starts, and use that as the reference.
		BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
		imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		imuParams.calibrationDataFile = "BNO055IMUCalibration.json";
		IMU.initialize(imuParams);
       /*
       //Sets drive system's motors directions
       leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
       leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
       rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
       rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
       */
		//Sets claw's init positions
		clawH.setPosition(-1);
		clawV.setPosition(1);
		claw.setPosition(1);

		//Sets hooks's init positions
		hook.setPosition(0);

		//Sets hooks's init positions
		clampH.setPosition(1);
		clampV.setPosition(-1);

		capstone.setPosition(0);

		//initConstants();
		preStartInitialization();
		postStartInitialization();
		conditionSticks();

	}

	@Override
	public void loop() {

		recomputeHeading();

		opModeIsActiveCount = 1;

		//Linear slide system
		double inputLift = gamepad2.left_stick_y;

		//Linear slide left and right movement system
		double inputLiftH = gamepad2.right_stick_y;

		//Intake system
		double inputBrickIn = gamepad1.right_trigger; //Intakes inwards
		double inputBrickOut = gamepad1.left_trigger; //Intakes outwards

		//Stone rotation system
		boolean inputClamp180 = gamepad2.x; //Rotates the clamp horizontally
		boolean inputClamp90 = gamepad2.b; //Rotates the clamp horizontally
		boolean inputClamp0 = gamepad2.y; //Rotates the clamp horizontally
		boolean inputClampV = gamepad2.a; //Rotates the clamp vertically

		//Claw grabbing system
		boolean inputClawH = gamepad1.left_bumper; //Claw lowered down
		boolean inputClawH2 = gamepad1.b;
		boolean inputClawV = gamepad1.right_bumper; //Claw lifted up
		boolean inputClaw = gamepad1.a; //Opens clamp

		//Foundation latching system
		boolean inputHook = gamepad1.x; //Claw lowered down

		//Capstone dropper
		boolean inputCapstone = gamepad1.y;

		//Dpad up and down movement system
		boolean inputDriveForward = gamepad1.dpad_up; //Drives forward
		boolean inputDriveBackward = gamepad1.dpad_down; //Drives backward

		//Dpad Strafe system
		boolean inputStrafeRight = gamepad1.dpad_right; //Strafes right
		boolean inputStrafeLeft = gamepad1.dpad_left; //Strafes left

		//Control stick drive system
		double leftStickX = gamepad1.left_stick_x; //Strafing system
		double rightStickX = -gamepad1.right_stick_x; //Turning system
		double leftStickY = gamepad1.left_stick_y; //Driving system
		double rightStickY = gamepad1.right_stick_y; //Nothing

		//Start system
		boolean inputStart = gamepad1.start; //Stops actions when pressed

		//Making inputs for the actions
		//Inputs for vertical linear slide system
		slide(-inputLift);

		//Inputs for horizontal linear slide system
		slideH(-inputLiftH);

		//Inputs intake system
		intake(inputBrickIn, inputBrickOut);

		stoneClamp(inputClamp0, inputClamp90, inputClamp180, inputClampV);

		//Inputs claw grabbing system
		claw(inputClawH, inputClawH2, inputClawV, inputClaw);

		//Inputs the hook system
		hook(inputHook);

		//Inputs the capstone dropper
		capstone(inputCapstone);

		//Inputs dpad up and down movement system
		drive(inputDriveForward, inputDriveBackward, inputStrafeRight, inputStrafeLeft);

		//Inputs control stick drive system
		//driveMecanum(forward, horizontal, turning);

		start(inputStart, inputClamp90, inputClaw);

		mecamumDrive(rightStickX, leftStickX, leftStickY, rightStickY);

	}

	//Function for the linear slide system
	public static void slide ( double inputLift){

		double Lift = (inputLift);

		double[] slidePowers = {Math.abs(Lift)};
		Arrays.sort(slidePowers);

		double biggestInput = slidePowers[0];
		if (biggestInput > 1) {
			Lift /= biggestInput;
		}

		slideL.setPower(-Lift);
		slideR.setPower(Lift);

	}

	//Function for the linear slide system
	public static void slideH ( double inputLiftH) {

		//Function for the linear slide system

		slideH.setPower(inputLiftH);

	}

	//Function for the intake system
	private void intake ( double inputBrickIn, double inputBrickOut){

		if (inputBrickIn != 0 && inputBrickOut <= 0) {

			intakeLM.setPower(-inputBrickIn);
			intakeRM.setPower(inputBrickIn);

			intakeLS.setPower(inputBrickIn);
			intakeRS.setPower(-inputBrickIn);

		}

		if (inputBrickOut != 0 && inputBrickIn <= 0) {

			intakeLM.setPower(inputBrickOut);
			intakeRM.setPower(-inputBrickOut);

			intakeLS.setPower(-inputBrickOut);
			intakeRS.setPower(inputBrickOut);

		}

		if (inputBrickOut <= 0 && inputBrickIn <= 0) {

			intakeRM.setPower(0);
			intakeLM.setPower(0);

			intakeLS.setPower(0);
			intakeRS.setPower(0);
		}
	}

	private void stoneClamp ( boolean inputClamp0, boolean inputClamp90, boolean inputClamp180, boolean inputClampV){

		if (startCount == 0) {

			if (inputClamp0 && clampCount0 == 0) {

				clampH.setPosition(1);

				clampCount0 = 1;
				clampCount90 = 0;
				clampCount180 = 0;

			}

			if (!inputClamp0 && clampCount0 == 1) {

				clampH.setPosition(1);

				clampCount0 = 0;

			}

			if (inputClamp90 && clampCount90 == 0) {

				clampH.setPosition(0.5);

				clampCount90 = 1;

			}

			if (!inputClamp90 && clampCount90 == 1) {

				clampH.setPosition(0.5);

				clampCount90 = 2;

			}

			if (inputClamp90 && clampCount90 == 2) {

				clampH.setPosition(1);

				clampCount90 = 3;

			}

			if (!inputClamp90 && clampCount90 == 3) {

				clampH.setPosition(1);

				clampCount90 = 0;

			}

			if (inputClamp180 && clampCount180 == 0) {

				clampH.setPosition(-1);

				clampCount180 = 1;

			}

			if (!inputClamp180 && clampCount180 == 1) {

				clampH.setPosition(-1);

				clampCount180 = 2;

			}

			if (inputClamp180 && clampCount180 == 2) {

				clampH.setPosition(1);

				clampCount180 = 3;

			}

			if (!inputClamp180 && clampCount180 == 3) {

				clampH.setPosition(1);

				clampCount180 = 0;

			}

			if (inputClampV && clampCountV == 0) {

				clampV.setPosition(0.425);

				clampCountV = 1;

			}

			if (!inputClampV && clampCountV == 1) {

				clampV.setPosition(0.425);

				clampCountV = 2;

			}

			if (inputClampV && clampCountV == 2) {

				clampV.setPosition(-1);

				clampCountV = 3;

			}

			if (!inputClampV && clampCountV == 3) {

				clampV.setPosition(-1);

				clampCountV = 0;

			}

		}

	}

	//Function for the claw grabbing system
	private void claw ( boolean inputClawH, boolean inputClawH2, boolean inputClawV, boolean inputClaw){

		if (inputClawH2){

			clawH.setPosition(1);

		}
		if (inputClawH){

			clawH.setPosition(0);

		}
		if (inputClawV && clawVCount == 0) {

			clawV.setPosition(-1);

			clawVCount = 1;

		}

		if (!inputClawV && clawVCount == 1) {

			clawV.setPosition(-1);

			clawVCount = 2;

		}

		if (inputClawV && clawVCount == 2) {

			clawV.setPosition(1);

			clawVCount = 3;

		}

		if (!inputClawV && clawVCount == 3) {

			clawV.setPosition(1);

			clawVCount = 0;
		}

		if (inputClaw && clawCount == 0) {

			claw.setPosition(1);

			clawCount = 1;

		}

		if (!inputClaw && clawCount == 1) {

			claw.setPosition(1);

			clawCount = 2;

		}

		if (inputClaw && clawCount == 2) {

			claw.setPosition(0.75);

			clawCount = 3;

		}

		if (!inputClaw && clawCount == 3) {

			claw.setPosition(0.75);

			clawCount = 0;

		}

	}

	//Function for the hook system
	private void hook ( boolean inputHook){

		if (inputHook && hookCount == 0) {

			hook.setPosition(0);

			hookCount = 1;

		}

		if (!inputHook && hookCount == 1) {

			hook.setPosition(0);

			hookCount = 2;

		}

		if (inputHook && hookCount == 2) {

			hook.setPosition(1);

			hookCount = 3;

		}

		if (!inputHook && hookCount == 3) {

			hook.setPosition(1);

			hookCount = 0;

		}

	}

	//Drops Capstone
	private void capstone ( boolean inputCapstone){

		if (inputCapstone && capstoneCount == 0) {

			capstone.setPosition(1);

			capstoneCount = 1;

		}

		if (!inputCapstone && capstoneCount == 1) {

			capstone.setPosition(1);

			capstoneCount = 2;

		}

		if (inputCapstone && capstoneCount == 2) {

			capstone.setPosition(0);

			capstoneCount = 3;

		}

		if (!inputCapstone && capstoneCount == 3) {

			capstone.setPosition(0);

			capstoneCount = 0;

		}

	}

	//Function for the dpad up and down movement system
	private void drive ( boolean inputDriveBackward, boolean inputDriveForward, boolean inputStrafeRight, boolean inputStrafeLeft) {
   /*
           if (inputDriveForward) {

               setDrive(0, 0, -100);

           }

           if (inputDriveBackward) {

               setDrive(0, 0, 100);

           }

           if (inputStrafeRight) {

               setDrive(0, -100, 0);

           }

           if (inputStrafeLeft) {

               setDrive(0, 100, 0);

           }

           if (!inputDriveForward && !inputDriveBackward && !inputStrafeRight && !inputStrafeLeft) {
               double error = expectedHeading - heading;
               setDrive(0, 0, error);

           }
           */
	}


	private void start ( boolean inputStart, boolean inputClamp90, boolean inputClaw){

		if (inputStart && inputClamp90 || inputClaw) {

			startCount = 1;

		}

		if (!inputStart && !inputClamp90 || !inputClaw) {

			startCount = 0;

		}
	}

	private void mecamumDrive(double rightStickX, double leftStickX, double leftStickY, double rightStickY) {
		if (rightStickX == 0.0 && leftStickX == 0.0 && leftStickY == 0.0 && rightStickY == 0.0) {
			// the left X (rotation) is within the deadband - the robot is not turning. If the robot was previously turning
			// then we reset the expected heading to the current heading and continue from there
			if (inDrive && inTurn && inStrafe) {
				expectedHeading = heading;
				inDrive = false;
				inTurn = false;
				inStrafe = false;

				mecamumDriveCount = 0;
			}
			// get the heading error for use in the heading correction PID loop - we only us the P part here.
			double max = Math.abs(rightStickY) + Math.abs(rightStickX);
			double error = expectedHeading - heading;
			setDrive(leftStickY, rightStickX, max * kp * error);
		} else {

			setDrive(rightStickX, leftStickX, leftStickY);

			//if (leftStickY > 0) {

			inDrive = true;
			//setDrive(rightStickX, 0 * leftStickX, 0 * leftStickY);

			//}

			//if (rightStickX > 0) {

			inTurn = true;
			//setDrive(0 * rightStickX, 0 * leftStickX, leftStickY);

			//}

			//if (leftStickX > 0) {

			inStrafe = true;
			//setDrive(0 * rightStickX, leftStickX, 0 * leftStickY);

			//}

			mecamumDriveCount = 1;
		}
	}
}