package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Drive {
    private DcMotor mFL;
    private DcMotor mBL;
    private DcMotor mFR;
    private DcMotor mBR;
    private double power;

    public Drive(DcMotor mFL, DcMotor mBL, DcMotor mFR, DcMotor mBR) {
        this.mFL = mFL;
        this.mBL = mBL;
        this.mFR = mFR;
        this.mBR = mBR;
    }

    public void DriveForward(double power){
        mFL.setPower(power);
        mBL.setPower(power);
        mFR.setPower(power);
        mBR.setPower(power);
    }

    public void DriveBackwards(double power, int time) throws InterruptedException {
        mFL.setPower(-power);
        mBL.setPower(-power);
        mFR.setPower(-power);
        mBR.setPower(-power);
        Thread.sleep(time * 1000);
        StopMotors();
    }

    public void DriveLeft(double power){
        mFL.setPower(-power);
        mBL.setPower(power);
        mFR.setPower(power);
        mBR.setPower(-power);
    }

    public void DriveRight(double power){
        mFL.setPower(power);
        mBL.setPower(-power);
        mFR.setPower(-power);
        mBR.setPower(power);
    }

    public void TurnRight(double power, long time) throws InterruptedException {
        mFL.setPower(power);
        mBL.setPower(power);
        mFR.setPower(-power);
        mBR.setPower(-power);
        Thread.sleep(time);
        StopMotors();
    }

    public void TurnLeft(double power){
        mFL.setPower(-power);
        mBL.setPower(-power);
        mFR.setPower(power);
        mBR.setPower(power);
    }

    public void DriveDiagonalUpLeft(double power){
        mBL.setPower(power);
        mFR.setPower(power);
    }

    public void DriveDiagonalUpRight(double power){
        mFL.setPower(power);
        mBR.setPower(power);
    }

    public void DriveDiagonalDownLeft(double power){
        mFL.setPower(-power);
        mBR.setPower(-power);
    }
    public void DriveDiagonalDownRight(double power){
        mBL.setPower(-power);
        mFR.setPower(-power);
    }

    public void StopMotors(){
        mFL.setPower(0);
        mBL.setPower(0);
        mFR.setPower(0);
        mBR.setPower(0);
   }
}
