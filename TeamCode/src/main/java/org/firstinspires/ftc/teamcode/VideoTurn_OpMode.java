package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains the testing of the elevator to go up for 5 seconds stop and go down for 5 seconds.
 */

@TeleOp(name="Video Turn OpMode", group="Tests")
public class VideoTurn_OpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor_f;
    private DcMotor leftMotor_b;
    private DcMotor rightMotor_f;
    private DcMotor rightMotor_b;
    private double power = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
		leftMotor_f  = hardwareMap.get(DcMotor.class, "mFL");
        leftMotor_b  = hardwareMap.get(DcMotor.class, "mBL");
        rightMotor_f  = hardwareMap.get(DcMotor.class, "mFR");
        rightMotor_b  = hardwareMap.get(DcMotor.class, "mBR");


        // Setting directions for the motors.
		rightMotor_b.setDirection(DcMotor.Direction.REVERSE);
        rightMotor_f.setDirection(DcMotor.Direction.REVERSE);
        leftMotor_b.setDirection(DcMotor.Direction.FORWARD);
        leftMotor_f.setDirection(DcMotor.Direction.FORWARD);



        //Building the elevator constructor.
		Drive motors = new Drive(leftMotor_f, leftMotor_b, rightMotor_f, rightMotor_b);
		
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
		motors.TurnRight(power, 8740);
    }
}
