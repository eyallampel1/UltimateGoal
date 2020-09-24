package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Elevator {
    private DcMotor mMain;
    private double power;

    public Elevator(DcMotor mMain, double power) {
        this.mMain = mMain;
        this.power = power;
    }

    public void testUp(double power) throws InterruptedException {
        mMain.setPower(-power);
        waitFor(3);
        StopElevator();
    }

    public void testDown(double power) throws InterruptedException {
        mMain.setPower(power);
        waitFor(3);
        StopElevator();
    }

    public void testUpDown(double power) throws InterruptedException {
        mMain.setPower(-power);
        waitFor(3);
        StopElevator();
        waitFor(1);
        mMain.setPower(power);
        waitFor(3);
        StopElevator();
    }

    public void StopElevator(){
        mMain.setPower(0);
   }

   private void waitFor(int sec) throws InterruptedException {
        Thread.sleep(sec * 1000);
   }
}
