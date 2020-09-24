package org.firstinspires.ftc.teamcode;

import android.widget.Toast;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains the testing of the elevator to go down for 5 seconds.
 */

@TeleOp(name="Basic: Down Elevator OpMode", group="Tests")
public class ElevatorDown_OpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor mainMotor = null;
	private double power = 0.3;
	
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
		mainMotor  = hardwareMap.get(DcMotor.class, "mFL");

        // Setting directions for the motors.
		mainMotor.setDirection(DcMotor.Direction.REVERSE);
		
		//Building the elevator constructor.
		Elevator elevator = new Elevator(mainMotor, power);
		
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
		elevator.testDown(power);
    }
}
