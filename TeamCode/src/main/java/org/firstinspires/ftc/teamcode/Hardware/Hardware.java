package org.firstinspires.ftc.teamcode.Hardware;

import android.view.View;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {

	public static final double MID_SERVO       =  0.5 ;
	public static final double ARM_UP_POWER    =  0.45 ;
	public static final double ARM_DOWN_POWER  = -0.45 ;

	/* local OpMode members. */
	private ElapsedTime period  = new ElapsedTime();

	//The drive system's motors are set to null
	HardwareMap hwMap = null;

	//The horizontal linear slide system's motors are set to null
	public DcMotor slideL = null; //Linear slide left motor
	public DcMotor slideR = null; //Linear slide right motor

	//The vertical linear slide system's motors are set to null
	public CRServo slideH = null; //Linear slide horizontal

	//The intake system's motors are set to null
	public DcMotor intakeLM = null; //Intake left motor
	public DcMotor intakeRM = null; //Intake right motor

	//The intake system's continuous servos are set to null
	public CRServo intakeLS = null; //Intake right servo
	public CRServo intakeRS = null; //Intake left servo

	//The claw rotate system's servos are set to null
	public Servo clawH = null; //Claw rotates the claw horizontally
	public Servo clawV = null; //Claw rotates the claw vertically

	//The claw grab system's servos are set to null
	public Servo claw = null; //Claw opens and closes

	//The foundation hook system's servos are set to null
	public Servo hook = null; //Foundation hook

	public Servo capstone = null; //Capstone dropper

	//The capstone clamp system's servos are set to null
	public Servo clampH = null; //Stone clamp horizontal
	public Servo clampV = null; //Stone clamp vertical

	public DcMotor leftFrontWheel = null; //Left front wheel
	public DcMotor leftBackWheel = null; //Left back wheel
	public DcMotor rightFrontWheel = null; //Right front wheel
	public DcMotor rightBackWheel = null; //Right back wheel

	NormalizedColorSensor colorSensor;
	View relativeLayout;
	DistanceSensor sensorRange;


	//Constructor
	//public Hardware(){}

	public void init(HardwareMap ahwMap) {

		// Save reference to Hardware map
		hwMap = ahwMap;

		//Gets the vertical linear slide's system from hardware
		slideL = hwMap.dcMotor.get("SlideL"); //Linear slide left motor
		slideR = hwMap.dcMotor.get("SlideR"); //Linear slide right motor

		//Gets the horizontal linear slide's system from hardware
		slideH = hwMap.crservo.get("SlideH"); //Linear slide horizontal

		//Gets the intake system's motors from hardware
		intakeLM = hwMap.dcMotor.get("IntakeLM"); //Intake left motor
		intakeRM = hwMap.dcMotor.get("IntakeRM"); //Intake right motor

		//Gets the intake system's continuous servos from hardware
		intakeLS = hwMap.crservo.get("IntakeLS");
		intakeRS = hwMap.crservo.get("IntakeRS");

		//Gets the claws system's servos from hardware
		clawH = hwMap.servo.get("ClawH"); //Claw rotates the claw vertically
		clawV = hwMap.servo.get("ClawV"); //Claw rotates the claw vertically
		claw = hwMap.servo.get("Claw"); //Claw opens and closes

		//Gets the hook system's servos from hardware
		hook = hwMap.servo.get("Hook"); //Foundation hook

		capstone = hwMap.servo.get("Capstone"); //Capstone dropper

		//Gets the capstone system's servos from hardware
		clampH = hwMap.servo.get("ClampH"); //Stone clamp horizontal
		clampV = hwMap.servo.get("ClampV"); //Stone clamp vertical

		//Gets the drive system's motors from hardware
		leftFrontWheel = hwMap.dcMotor.get("LeftFrontMotor"); //Left front wheel
		leftBackWheel = hwMap.dcMotor.get("LeftBackMotor"); //Left back wheel
		rightFrontWheel = hwMap.dcMotor.get("RightFrontMotor"); //Right front wheel
		rightBackWheel = hwMap.dcMotor.get("RightBackMotor"); //Right back wheel

		leftFrontWheel  = hwMap.get(DcMotor.class, "LeftFrontMotor");
		leftBackWheel = hwMap.get(DcMotor.class, "LeftBackMotor");
		rightFrontWheel  = hwMap.get(DcMotor.class, "RightFrontMotor");
		rightBackWheel = hwMap.get(DcMotor.class, "RightBackMotor");

		//Sets drive system's motors directions
		leftFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
		leftBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);
		rightFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
		rightBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);

		// get a reference to the color sensor.
		colorSensor = hwMap.get(NormalizedColorSensor.class, "ColorSensor");

		// get a reference to the distance sensor that shares the same name.
		sensorRange = hwMap.get(DistanceSensor.class, "DistanceSensor");

		leftFrontWheel.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
		leftBackWheel.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
		rightFrontWheel.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
		rightBackWheel.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

		// Set all motors to zero power
		leftFrontWheel.setPower(0);
		leftBackWheel.setPower(0);
		rightFrontWheel.setPower(0);
		rightBackWheel.setPower(0);

		// Set all motors to run without encoders.
		// May want to use RUN_USING_ENCODERS if encoders are installed.
		leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       /*
       //Sets claw's init positions
       claw.setPosition(-1);
       claw.setPosition(-1);
       claw.setPosition(1);

       //Sets hooks's init positions
       hookL.setPosition(1);
       hookR.setPosition(1);

       clampH.setPosition(1);
       clampV.setPosition(1);
       */
	}
}