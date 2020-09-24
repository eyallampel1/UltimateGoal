package org.firstinspires.ftc.teamcode;

import android.os.PowerManager;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file contains an example of an gamepad OpMode made by Elior Yousefi.
 */
@TeleOp(name="GabayGamePadTest", group="Tests")
public class GabayGamePadTest extends  OpMode{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    // private DistanceSensor dsB = null; --ITS A TOUCH SENSOR
     private DcMotor mbl=null;
     private DcMotor mbr=null;
     private DcMotor mfl=null;
     private DcMotor mfr=null;
     private double power = 0.5;

        /*
         * Code to run ONCE when the driver hits INIT
         */
        @Override
        public void init() {
            telemetry.addData("Status", "Initialized");

            // Initializing the motors here.
           mbl=hardwareMap.get(DcMotor.class,"mBL");
           mbr =hardwareMap.get(DcMotor.class,"mBR");
           mfl=hardwareMap.get(DcMotor.class,"mFL");
           mfr=hardwareMap.get(DcMotor.class,"mFR");
            // Initializing the color sensors here.
            // Initializing the distance sensors here.
            //dsB = hardwareMap.get(DistanceSensor.class, "dsB"); --ITS A TOUCH SENSOR

            // methods associated with the Rev2mDistanceSensor class.
            // Rev2mDistanceSensor sensorTimeOfFlightB = (Rev2mDistanceSensor)dsB; --ITS A TOUCH SENSOR

            // Reverse the motors who are backwards.
            mfr.setDirection(DcMotor.Direction.FORWARD);
            mbr.setDirection(DcMotor.Direction.REVERSE);
            mfl.setDirection(DcMotor.Direction.FORWARD);
            mbl.setDirection(DcMotor.Direction.REVERSE);
            // Turning on the color sensors's led

            // Tell the driver that initialization is complete.
            telemetry.addData("Status", "Initialized");
        }

        /*
         * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
         */
        @Override
        public void init_loop() {
                  }

        /*
         * Code to run ONCE when the driver hits PLAY
         */
        @Override
        public void start() {
            runtime.reset();
        }

        /*
         * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
         */
        @Override
        public void loop() {
            // Setup a variable for each drive wheel.
            double MFL;
            double MFR;
            double MBL;
            double MBR;

            // if we press the left joystick or right stick while moving it so the bot will "sprint" (power to 0.9).
            if(gamepad1.left_stick_button || gamepad1.right_stick_button){
                power = 0.9;
            }else{
                power = 0.5;
            }
            double driveX = gamepad1.left_stick_x;
            double driveY = gamepad1.left_stick_y;
            double turnX  =  gamepad1.right_stick_x;
            MFL=Range.clip(driveY-driveX -turnX,-power, power);
            MBL=Range.clip(-driveY-driveX+turnX,-power,power);
            MFR=Range.clip(-driveY-driveX-turnX,-power,power);
            MBR=Range.clip(driveY-driveX+turnX,-power,power);

            // Here we declare the gamepad values given from the gamepad.
            // After we use range.clip(which sets a minimum and maximum for the gamepad values.
            // And lastly by the mechanum wheels gamepad formula we give every wheel a value.

            // Send calculated power to wheels
            mbl.setPower(MBL);
            mbr.setPower(MBR);
            mfl.setPower(MFL);
            mfr.setPower(MFR);
            // Show the elapsed game time, wheel power, rgb from sensors.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Front (%.2f), right (%.2f)", MFL,MFR);
            telemetry.addData("Motors", "Back (%.2f), right (%.2f)", MBL, MBR);
        }

        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop() {
        }

    }
