package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains the testing of the elevator to go up for 5 seconds.
 */

@TeleOp(name="Basic: Video Elevator OpMode", group="Tests")
public class ElevatorVideo_OpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor mainMotor = null;
	private double power = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
		mainMotor  = hardwareMap.get(DcMotor.class, "mFL");

        // Setting directions for the motors.
		mainMotor.setDirection(DcMotor.Direction.REVERSE);
		
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()){
            if(gamepad1.y){
                mainMotor.setPower(power);
            }else if(gamepad1.a){
                mainMotor.setPower(-power);
            }else{
                mainMotor.setPower(0);
            }
        }
    }
}
