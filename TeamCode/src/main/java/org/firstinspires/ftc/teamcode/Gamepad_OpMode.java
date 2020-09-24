/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an example of a test for the robot gamepad.
 */

@TeleOp(name="Gamepad OpMode", group="Gamepad")
public class Gamepad_OpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotor_f = null;
    private DcMotor leftMotor_b = null;
    private DcMotor rightMotor_f = null;
    private DcMotor rightMotor_b = null;
    private DistanceSensor dsF;
    private DistanceSensor dsB;
    private DistanceSensor dsR;
    //private DistanceSensor dsL;
    private double power = 0.35;
    private BNO055IMU imu;
    private Orientation angles;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables.
        leftMotor_f  = hardwareMap.get(DcMotor.class, "mFL");
        leftMotor_b = hardwareMap.get(DcMotor.class, "mBL");
        rightMotor_f  = hardwareMap.get(DcMotor.class, "mFR");
        rightMotor_b = hardwareMap.get(DcMotor.class, "mBR");

        // Reversing the left motors.
        leftMotor_f.setDirection(DcMotor.Direction.FORWARD);
        leftMotor_b.setDirection(DcMotor.Direction.REVERSE);
        rightMotor_f.setDirection(DcMotor.Direction.FORWARD);
        rightMotor_b.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        // Initialize the hardware variables.

        imu  = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {
        double lGP_X, lGP_Y, rGP_X;//  lGP_Y; If needed
        lGP_X = gamepad1.left_stick_x;
        lGP_Y = gamepad1.left_stick_y;
        rGP_X = gamepad1.right_stick_x / 2;
        //lGP_Y = gamepad1.right_stick_y; if needed
        // the diagonal motors will always have the same power for the basic moving and straifing

        if(gamepad1.left_stick_button || gamepad1.right_stick_button){
            power = 0.85;
        }else{
            power = 0.35;
        }

        //Front
        double powerFleft = Range.clip((lGP_Y-lGP_X - rGP_X), -power, power); // front left motor
        double powerBleft = Range.clip((-lGP_Y - lGP_X + rGP_X), -power, power); // front right motor

        //Back
        double powerFright = Range.clip((-lGP_Y - lGP_X - rGP_X), -power, power); // back left motor
        double powerBright = Range.clip((lGP_Y - lGP_X + rGP_X), -power, power); // back right motor

        //Front setPower
        leftMotor_f.setPower(powerFleft);
        rightMotor_f.setPower(powerFright);

        //Left SetPower
        leftMotor_b.setPower(powerBleft);
        rightMotor_b.setPower(powerBright);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        telemetry.addData("Heading:", angles.firstAngle);
        telemetry.addData("Roll:", angles.secondAngle);
        telemetry.addData("Pitch:", angles.thirdAngle);
        //telemetry.addData("range", String.format("%.01f cm", dsL.getDistance(DistanceUnit.CM)));
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "Front: left (%.2f), right (%.2f)", powerFleft, powerFright);
        telemetry.addData("Motors", "Back: left (%.2f), right (%.2f)", powerBleft, powerBright);
        telemetry.addData("Power", "Power: main (%.2f)", power);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
