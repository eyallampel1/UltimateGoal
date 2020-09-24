package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.videoio.VideoCapture;

@TeleOp(name="Color Detection", group="Tests")
public class ColorDetction extends LinearOpMode {
    public void detectColor() {
        final Mat original_img = new Mat();

        Thread myT = new Thread(new Runnable() {
            @Override
            public void run() {
                VideoCapture cap = new VideoCapture();
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                cap.open(0);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                cap.read(original_img);
                if (cap.isOpened()) {
                    System.out.println("camera open");
                    while (opModeIsActive()) {
                        cap.read(original_img);
                        //   Highgui.imwrite("D:\\YY.bmp",original_img);
                        Mat original_img2 = original_img.clone();
                        Mat original_img3 = original_img.clone();
                        Mat gray = original_img.clone();
                        Mat thr = original_img.clone();
                        ///detect Color

                        //convert to hsv
                        //     Imgproc.cvtColor(original_img, original_img,Imgproc.COLOR_BGR2HSV);

                        //  Core.inRange(original_img,new Scalar(170,5,5),new Scalar(180,254,254),original_img2);//pink
                        //      Core.inRange(original_img,new Scalar(0,5,5),new Scalar(1,254,254),original_img2);//red
                        //   Core.inRange(original_img,new Scalar(81,1,1),new Scalar(83,254,254),original_img2);//red
                        Imgproc.blur(original_img, original_img, new Size(3, 3));
                        try {
                            Core.inRange(original_img, new Scalar(5, 100, 100), new Scalar(71, 255, 500), original_img);//pink
                        } catch (NumberFormatException ex) {

                            Core.inRange(original_img, new Scalar(1, 1, 1), new Scalar(254, 254, 254), original_img2);//red
                        }
                        Mat afterInRange = original_img2.clone();
//                        Imgproc.dilate(original_img2, original_img2, new Mat());
//                        List<MatOfPoint> points = new ArrayList<>();
//
//                        Imgproc.findContours(original_img2, points, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//                        //DARW COUNTOR
//                        //    Imgproc.drawContours(original_img2,points,6,new Scalar(255,0,0),2);
//
//                        for (int idx = 0; idx < points.size(); idx++) {
//                            Imgproc.drawContours(original_img3, points, idx, new Scalar(255, 0, 0), 8);
//                        }

                        Double cannyThreshHoldD = 0.0;
                        try {
                            cannyThreshHoldD = 10.0;
                        } catch (NumberFormatException ex) {
                            cannyThreshHoldD = 0.0;
                        }


                        Imgproc.Canny(original_img2, original_img2, cannyThreshHoldD, cannyThreshHoldD * 3);
                        Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
                        Imgproc.dilate(original_img2, original_img2, element);

                        Core.addWeighted(original_img3, 0.5, original_img, 0.5, 0.0, original_img);
                        boolean redLed = false;
                        double blackWhitePixel;
                        int whitePixelFramCounter = 0;


                        Core.MinMaxLocResult mmr = Core.minMaxLoc(original_img2);
                        Point matchLoc;
                        matchLoc = mmr.minLoc;

                        for (int rows = 0; rows < original_img.rows(); rows++) {
                            for (int cols = 0; cols < original_img.cols(); cols++) {
                                blackWhitePixel = original_img2.get(rows, cols)[0];
                                if (blackWhitePixel != 0.0) {
                                    //start counting if you count 50 then foundMatch!
                                    whitePixelFramCounter += 1;
                                    if (whitePixelFramCounter == 5000) {
                                        System.out.println(blackWhitePixel);
                                        System.out.println("row=" + rows + " cols=" + cols);
                                        //Imgproc.drawMarker(original_img, new Point(cols + 30, rows + 70), new Scalar(255, 0, 0), 9, 9, 20, 3);

                                        //find center of mass
                                        Imgproc.threshold( original_img2, thr, 100,255,Imgproc.THRESH_BINARY);
                                        Moments m = Imgproc.moments(thr,true);
                                        Point p=new Point(m.m10/m.m00, m.m01/m.m00);
                                        Imgproc.putText(original_img,"RED",p,4,3.0,new Scalar(0,0,255),3);
                                        //end find center of mass

                                        redLed = true;

                                    } else {
                                        if (redLed) {

                                            redLed = false;
                                        }

                                    }

                                }

                            }
                        }

                        cap.read(gray);
                        //Core.drawMarker(original_img3,new Point(200,200),new Scalar(255,0,0),9,90,20,3);//draw +

                        // Mat afterCannyAnd_AndConverToHSV=myOpenCVobject.rgb2hsv(original_img2);
                        //======================================================///////////play to screen


                        //    Image myimage4 = myOpenCVobject.matToImage(original_img4);
                        //     myIMG4.setPreserveRatio(true);
                        //    myIMG4.setImage(myimage4);
                        //   cap.release();

                    }
                } else {
                    System.out.println("not opened");
                }

            }
        });

        myT.start();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        detectColor();
    }
}
