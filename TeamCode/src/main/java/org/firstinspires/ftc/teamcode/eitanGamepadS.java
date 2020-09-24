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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.ColorSensor;
/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="EitanGmePad", group="tests")
@Disabled
public class eitanGamepadS extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor_f = null;
    private DcMotor leftMotor_b = null;
    private DcMotor rightMotor_f = null;
    private DcMotor rightMotor_b = null;
    private DistanceSensor dsF;
    private BNO055IMU imu;
    private double power = 0.35;
    private Orientation angles;
    ColorSensor color_sensor;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app o n the phone).
        leftMotor_f  = hardwareMap.get(DcMotor.class, "mFL");
        leftMotor_b = hardwareMap.get(DcMotor.class, "mFR");
        rightMotor_f = hardwareMap.get(DcMotor.class, "mFR");
        rightMotor_b = hardwareMap.get(DcMotor.class, "mFL");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor_f.setDirection(DcMotor.Direction.FORWARD);
        leftMotor_b.setDirection(DcMotor.Direction.FORWARD);
        rightMotor_f.setDirection(DcMotor.Direction.REVERSE);
        rightMotor_b.setDirection(DcMotor.Direction.REVERSE);
        dsF = hardwareMap.get(DistanceSensor.class, "sensor_range");
        Rev2mDistanceSensor sensorTimeOfFlightF = (Rev2mDistanceSensor)dsF;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu  = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        color_sensor = hardwareMap.colorSensor.get("color");


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
        // Setup a variable for each drive wheel to save power level for telemetry
        double lGP_X, lGP_Y, rGP_X;//  lGP_Y; If needed
        lGP_X = gamepad1.left_stick_x;
        lGP_Y = gamepad1.left_stick_y;
        rGP_X = gamepad1.right_stick_x / 2;
        color_sensor.enableLed(true);

        if(gamepad1.left_stick_button || gamepad1.right_stick_button){
            power = 0.85;
        }else{
            power = 0.35;
        }

        if (dsF.getDistance(DistanceUnit.CM)  <= 30){
            leftMotor_f.setPower(-power);
            rightMotor_f.setPower(-power);
            leftMotor_b.setPower(-power);
            rightMotor_b.setPower(-power);

        }
        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.


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
        // Show the elapsed game time and wheel power.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        telemetry.addData("Heading:", angles.firstAngle);
        telemetry.addData("Roll:", angles.secondAngle);
        telemetry.addData("Pitch:", angles.thirdAngle);
        telemetry.addData("range", String.format("%.01f cm", dsF.getDistance(DistanceUnit.CM)));
        telemetry.addData("RED", "color:",color_sensor.red());
        telemetry.addData("BLUE", "color:", color_sensor.blue());
        telemetry.addData("GREEN","color:", color_sensor.green());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "Front: left (%.2f), right (%.2f)", powerFleft, powerFright);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", powerBleft, powerBright);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}
