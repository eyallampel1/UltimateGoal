package org.firstinspires.ftc.teamcode;

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
@TeleOp(name="Elior OpMode", group="Tests")
public class EliorOpMode extends  OpMode{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor_f = null;
    private DcMotor leftMotor_b = null;
    private DcMotor rightMotor_f = null;
    private DcMotor rightMotor_b = null;
    private ColorSensor csFL = null;
    private ColorSensor csFR = null;
    private ColorSensor csBL = null;
    private ColorSensor csBR = null;
    private DistanceSensor dsF = null;
    private DistanceSensor dsL = null;
    private DistanceSensor dsR = null;
    // private DistanceSensor dsB = null; --ITS A TOUCH SENSOR
    private double power = 0.5;

        /*
         * Code to run ONCE when the driver hits INIT
         */
        @Override
        public void init() {
            telemetry.addData("Status", "Initialized");

            // Initializing the motors here.
            leftMotor_f  = hardwareMap.get(DcMotor.class, "mFL");
            leftMotor_b  = hardwareMap.get(DcMotor.class, "mBL");
            rightMotor_f  = hardwareMap.get(DcMotor.class, "mFR");
            rightMotor_b  = hardwareMap.get(DcMotor.class, "mBR");

            // Initializing the color sensors here.
            csFL  = hardwareMap.get(ColorSensor.class, "csFL");
            csFR  = hardwareMap.get(ColorSensor.class, "csFR");
            csBL  = hardwareMap.get(ColorSensor.class, "csBL");
            csBR  = hardwareMap.get(ColorSensor.class, "csBR");

            // Initializing the distance sensors here.
            dsF = hardwareMap.get(DistanceSensor.class, "dsF");
            dsR = hardwareMap.get(DistanceSensor.class, "dsR");
            dsL = hardwareMap.get(DistanceSensor.class, "dsL");
            //dsB = hardwareMap.get(DistanceSensor.class, "dsB"); --ITS A TOUCH SENSOR

            // methods associated with the Rev2mDistanceSensor class.
            Rev2mDistanceSensor sensorTimeOfFlightF = (Rev2mDistanceSensor)dsF;
            Rev2mDistanceSensor sensorTimeOfFlightR = (Rev2mDistanceSensor)dsR;
            Rev2mDistanceSensor sensorTimeOfFlightL = (Rev2mDistanceSensor)dsL;
            // Rev2mDistanceSensor sensorTimeOfFlightB = (Rev2mDistanceSensor)dsB; --ITS A TOUCH SENSOR

            // Reverse the motors who are backwards.
            leftMotor_f.setDirection(DcMotor.Direction.FORWARD);
            leftMotor_b.setDirection(DcMotor.Direction.REVERSE);
            rightMotor_f.setDirection(DcMotor.Direction.FORWARD);
            rightMotor_b.setDirection(DcMotor.Direction.REVERSE);

            // Turning on the color sensors's led
            csFL.enableLed(true);
            csFR.enableLed(true);
            csBL.enableLed(true);
            csBR.enableLed(true);

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
            double leftPower_f;
            double leftPower_b;
            double rightPower_f;
            double rightPower_b;

            // if we press the left joystick or right stick while moving it so the bot will "sprint" (power to 0.9).
            if(gamepad1.left_stick_button || gamepad1.right_stick_button){
                power = 0.9;
            }else{
                power = 0.5;
            }

            // Here we declare the gamepad values given from the gamepad.
            // After we use range.clip(which sets a minimum and maximum for the gamepad values.
            // And lastly by the mechanum wheels gamepad formula we give every wheel a value.
            double driveX = gamepad1.left_stick_x;
            double driveY = gamepad1.left_stick_y;
            double turnX  =  gamepad1.right_stick_x;
            leftPower_f = Range.clip(driveY - driveX - turnX, -power, power);
            leftPower_b   = Range.clip(-driveY - driveX + turnX, -power, power);
            rightPower_f = Range.clip(-driveY - driveX - turnX, -power, power);
            rightPower_b   = Range.clip(driveY - driveX + turnX, -power, power);

            // Send calculated power to wheels
            leftMotor_f.setPower(leftPower_f);
            leftMotor_b.setPower(leftPower_b);
            rightMotor_f.setPower(rightPower_f);
            rightMotor_b.setPower(rightPower_b);

            // Show the elapsed game time, wheel power, rgb from sensors.
            telemetry.addData("Color Sensors RED", "FrontRight (%.2f), FrontLeft (%.2f), BackRight (%.2f), BackLeft (%.2f)", csFR.red(), csFL.red(), csBR.red(), csBL.red());
            telemetry.addData("Color Sensors GREEN", "FrontRight (%.2f), FrontLeft (%.2f), BackRight (%.2f), BackLeft (%.2f)", csFR.green(), csFL.green(), csBR.green(), csBL.green());
            telemetry.addData("Color Sensors BlUE", "FrontRight (%.2f), FrontLeft (%.2f), BackRight (%.2f), BackLeft (%.2f)", csFR.blue(), csFL.blue(), csBR.blue(), csBL.blue());
            telemetry.addData("Color Sensors ALPHA", "FrontRight (%.2f), FrontLeft (%.2f), BackRight (%.2f), BackLeft (%.2f)", csFR.alpha(), csFL.alpha(), csBR.alpha(), csBL.alpha());
            telemetry.addData("Front Range CM", String.format("%.01f cm", dsF.getDistance(DistanceUnit.CM)));
            telemetry.addData("Front Range M", String.format("%.01f m", dsF.getDistance(DistanceUnit.METER)));
            // telemetry.addData("Back Range CM", String.format("%.01f cm", dsF.getDistance(DistanceUnit.CM))); --ITS A TOUCH SENSOR
            // telemetry.addData("Back Range M", String.format("%.01f m", dsF.getDistance(DistanceUnit.METER))); --ITS A TOUCH SENSOR
            telemetry.addData("Right Range CM", String.format("%.01f cm", dsR.getDistance(DistanceUnit.CM)));
            telemetry.addData("Right Range M", String.format("%.01f m", dsR.getDistance(DistanceUnit.METER)));
            telemetry.addData("Left Range CM", String.format("%.01f cm", dsL.getDistance(DistanceUnit.CM)));
            telemetry.addData("Left Range M", String.format("%.01f m", dsL.getDistance(DistanceUnit.METER)));
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Front (%.2f), right (%.2f)", leftPower_f, rightPower_f);
            telemetry.addData("Motors", "Back (%.2f), right (%.2f)", leftPower_b, rightPower_b);
        }

        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop() {
        }

    }
