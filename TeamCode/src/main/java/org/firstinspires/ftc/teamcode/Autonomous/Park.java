package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.MechBase.CustomAMecBase;

@Autonomous(name = "Park", group = "AMyMecBase")
public class Park extends CustomAMecBase {

	/* These classes were lost in importing process
	AutonomousFunctions AutoFunction = new AutonomousFunctions();

	IMUFunctions IMUFunction = new IMUFunctions();

	ColorDistanceSensor color = new ColorDistanceSensor();
	*/

	Hardware robot = new Hardware();

	// hsvValues is an array that will hold the hue, saturation, and value information.
	float hsvValues[] = {0F, 0F, 0F};

	// values is a reference to the hsvValues array.
	final float values[] = hsvValues;

	// sometimes it helps to multiply the raw RGB values with a scale factor
	// to amplify/attentuate the measured values.
	final double SCALE_FACTOR = 255;

	@Override
	public void runOpMode() {
		robot.init(hardwareMap);

		clawOpen();
		robot.clawV.setPosition(1);
		clawSide();
		telemetry.addData("Hue", hsvValues[0]);

		//AutoFunction.clawOpen();

		// initialize the control constants
		initConstants();
		// initialize the hardware before the opmode starts
		preStartInitialization();
		waitForStart();
		// the OpMode has started, do any initialization that needs to be postponed until the OpMode starts.
		postStartInitialization();
		// memove this simple autonomous program and add your autonomous program here

		slideUp(1300);

		slideDown(1300);

		move(20, 0, 1);

		telemetry.addData("Path Is Finished", "");
	}

	public void clawDownOpen(int Time) {

		robot.clawV.setPosition(-0.45);

		sleep(Time);

		robot.claw.setPosition(0.75);

	}

	public void clawSide() {

		robot.clawH.setPosition(0);

	}

	public void clawBack() {

		robot.clawH.setPosition(1);

	}

	public void clawUp() {

		robot.clawV.setPosition(0.5);

	}

	public void clawDown() {

		robot.clawV.setPosition(-.9);

	}

	public void clawOpen() {

		robot.claw.setPosition(0.75);

	}

	public void clawClose() {

		robot.claw.setPosition(0.95);

	}

	public void hookUp() {

		robot.hook.setPosition(1);

	}

	public void hookDown() {

		robot.hook.setPosition(0);

	}

	public void slideUp(int Time) {

		robot.slideL.setPower(-0.5);
		robot.slideR.setPower(0.5);

		sleep(Time);

		robot.slideL.setPower(0);
		robot.slideR.setPower(0);

	}

	public void slideDown(int Time) {

		robot.slideL.setPower(0.5);
		robot.slideR.setPower(-0.5);

		sleep(Time);

		robot.slideL.setPower(0);
		robot.slideR.setPower(0);

	}
   /*
   public void skystoneSense (int Time) {

       while (color.SenseCount < 1) {

           sleep(500);

           if (hsvValues[0] >= 50) {

               telemetry.addData("Target Found", "Skystone");

               sleep(Time);

               color.SenseCount = 2;

           }

           if (hsvValues[0] <= 50) {

               telemetry.addData("Target is Missing", "Stone");

               move(7, 0, 1);

               color.SenseCount = color.SenseCount + 1;

               sleep(Time);

           } else {

               telemetry.addData("Loading...", "");

               sleep(Time);

               color.SenseCount = 2;

           }

           /*
           function.clawOpen();
           function.clawDown();
           function.clawDownClose(1000);
            */
           /*
       }

       color.SenseCount = 0;

   }
   */

           /*
   public void skystoneSense (int Time) {

       while (color.SenseCount < 1) {

           sleep(500);

           if (hsvValues[0] >= 50) {

               telemetry.addData("Target Found", "Skystone");

               sleep(Time);

               color.SenseCount = 2;

           }

           if (hsvValues[0] <= 50) {

               telemetry.addData("Target is Missing", "Stone");

               move(7, 0, 1);

               color.SenseCount = color.SenseCount + 1;

               sleep(Time);

           } else {

               telemetry.addData("Loading...", "");

               sleep(Time);

               color.SenseCount = 2;

           }

           /*
           function.clawOpen();
           function.clawDown();
           function.clawDownClose(1000);
            */
           /*
       }

       color.SenseCount = 0;

   }
   */

}