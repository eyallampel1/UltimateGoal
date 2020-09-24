package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
@ -53,7 +73,7 @@ import org.opencv.videoio.VideoCapture;
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class FirstTestWebcam extends OpMode {

    VideoCapture cap=new VideoCapture();

    ElapsedTime runtime;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
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

        if(cap.isOpened()){
            /*Imgproc.matchTemplate(image, template, input, 0);
            Imgproc.matchTemplate(image, template, input, 0);
            Imgproc.matchTemplate(image, template, input, 0);
            Imgproc.matchTemplate(image, template, input, 0);
            Imgproc.matchTemplate(image, template, input, 0);
            */
        }else {
            telemetry.addLine("Camera disconnected, not a good idea...");
        }
    }

/*
 * Code to run ONCE after the driver hits STOP
 */

    @Override
    public void stop() {

    }

}


