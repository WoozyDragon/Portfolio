package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Core;
import org.opencv.imgproc.Moments;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.ArrayList;

@Autonomous

public class OctoAuton extends LinearOpMode {
    OpenCvWebcam webcam;
    DcMotor rightBack, leftBack, leftFront, rightFront;
    DcMotor leftLift, rightLift;
    Blinker control_Hub, expansion_Hub_2;
    BNO055IMU c_imu, e_imu;
    Servo roller;
    
    DistanceSensor front, distL, distR;
    
    double angle, rawAngle, offset;
    //public static int maxColor = 0;
    
    @Override
    public void runOpMode() {
        motorSetup();
        gyroSetup();
        miscSetup();
        webcamSetup();
        
        delay(3000);

        
        
        while (!isStarted() && !isStopRequested()) {
            opModeIsActive();
            telemetry.addLine("ready!");
            telemetry.addData("Distance", getDistance(front));
            telemetry.addData("Color", octoColor.maxColor);
            telemetry.addLine();
            telemetry.addLine("Colors: Heavy debug");
            telemetry.addData("Green", octoColor.green);
            telemetry.addData("Orange", octoColor.orange);
            telemetry.addData("Purple", octoColor.purple);
            telemetry.update();
        }
        
        //power angle time
        waitForStart();
        
        int maxColor = octoColor.maxColor;
        telemetry.addData("max color", maxColor);
        telemetry.update();

        lift(1, 1000);
        
        go_goAngle(0.125, 280);
        long forwardTime = getTime();
        while(getData(front) > 450 && getTime() - forwardTime <= 4000) {
            telemetry.addData("Front", getDistance(front));
            telemetry.update();
        }
        goAngle(0.125, 280, 250);
        delay(500);
        
        go_goAngle(0.25, 0);
        forwardTime = getTime();
        while(getDistance(front) > 170 && getTime() - forwardTime <= 1000) {
            telemetry.addData("Front", getDistance(front));
            telemetry.update();
        }
        stopDrive();

        //goAngle(0.5, 305, 850);
 
        delay(500);
        outtake(); 
        delay(500);
        goAngle(-0.5, 310, 1000);
        //goAngle(1, 90, 1000);
        
        delay(1000);
        
        telemetry.addData("max color", maxColor);
        telemetry.update();
        
        if (maxColor == 1) {
            goAngle(0.5, 0, 1550);
            delay(500);
            goAngle(0.5, 270, 1150);
        } else if (maxColor == 2) {
            goAngle(0.5, 0, 1750);
        } else if (maxColor == 3) {
            goAngle(0.5, 0, 1550);
            delay(500);
            goAngle(0.5, 90, 1500);
        }
        
        delay(250);
        goAngle(0.5, 0, 500);
        
        
        delay(1000);
        //ift(-1, 975);
        
        /*
        WOLCOTT WAY
        telemetry.addData("max color", octoColor.maxColor);
        telemetry.update();
        delay(1000);
        
        lift(1, 1000);
        goAngle(0.5, 305, 850);
        delay(500);
        outtake(); 
        delay(500);
        goAngle(-0.5, 310, 750);
        //goAngle(1, 90, 1000);
        
        delay(1000);
        
        if (octoColor.maxColor == 1) {
            goAngle(0.5, 0, 1500);
            delay(500);
            goAngle(0.5, 270, 1250);
        } else if (octoColor.maxColor == 2) {
            goAngle(0.5, 0, 1750);
        } else if (octoColor.maxColor == 3) {
            goAngle(0.5, 0, 1500);
            delay(500);
            goAngle(0.5, 90, 1400);
        }
        
        delay(250);
        goAngle(0.5, 0, 500);
        
        
        delay(1000);
        //ift(-1, 975);
        
        */
        /*
        lift(1, 1000);
        goAngle(0.5, 0, 500);
        outtake();
        */
    }
    
    void go_goAngle(double power, double angle) {
        angle *= Math.PI/180;
        angle += Math.PI;

        rightFront.setPower(power * Math.sin(angle - (Math.PI/4)));
        rightBack.setPower(power * Math.sin(angle + (Math.PI/4)));
        leftBack.setPower(power * Math.sin(angle - (Math.PI/4)));
        leftFront.setPower(power * Math.sin(angle + (Math.PI/4)));
    }
    
    void go_goAngle(double power, double angle, double left, double right) {
        angle *= Math.PI/180;
        angle += Math.PI;

        rightFront.setPower(-right + power * Math.sin(angle - (Math.PI/4)));
        rightBack.setPower(right + power * Math.sin(angle + (Math.PI/4)));
        leftBack.setPower(left + power * Math.sin(angle - (Math.PI/4)));
        leftFront.setPower(-left + power * Math.sin(angle + (Math.PI/4)));
    }
    
    void goAngle(double power, double angle, long time) {
        go_goAngle(power, angle);
        delay(time);
        stopDrive();
    }
    
    void forward(double power, long time) {
        long startTime = getTime();
        getAngle();
        double initialIMU = angle;
        
        while (getTime() - startTime < time) {
            getAngle();
            double error = angle - initialIMU;
            go_goAngle(power, 45, power*error/200, power*error/200);
        }
        
        stopDrive();
    }
    
    void go_turn(double power) {
        rightFront.setPower(-power);
        rightBack.setPower(power);
        leftBack.setPower(power);
        leftFront.setPower(-power);
    }
    
    void turn(double power, long time) {
        go_turn(power);
        delay(time);
        stopDrive();
    }
    
    void stopDrive() {
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        leftFront.setPower(0);
    }
    
    void go_lift(double power) {
        leftLift.setPower(power);
        rightLift.setPower(power);
    }
    
    void lift(double power, long time) {
        go_lift(power);
        delay(time);
        stopLift();
    }
    
    void stopLift() {
        leftLift.setPower(0);
        rightLift.setPower(0);
    }
    
    void intake() {
        roll(0.25, 400);
    }
    
    void outtake() {
        roll(0.75, 400);
    }
    
    void go_roll(double direction) {
        roller.setPosition(direction);
    }
    
    void roll(double direction, long time) {
        go_roll(direction);
        delay(time);
        go_roll(0.5);
    }
    
    double[] past = {0, 0, 0};
    double getDistance(DistanceSensor sensor) {
        past[2] = getData(sensor);
        past[1] = getData(sensor);
        past[0] = getData(sensor);
        
        return (past[2] + past[1] + past[0])/3;
    }
    
    double getData(DistanceSensor sensor) { 
        return sensor.getDistance(DistanceUnit.MM);
    }
    
    public void getAngle() {
        Orientation c_angles = c_imu.getAngularOrientation(
        AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        Orientation e_angles = e_imu.getAngularOrientation(
        AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        rawAngle = (c_angles.firstAngle + e_angles.firstAngle)/2;
        //rawAngle = c_angles.firstAngle;
        angle = rawAngle + offset;
    }
    
    //in milliseconds
    public static long getTime() {
        return (System.nanoTime()) / 1000000;
    }
    
    //sleep for ms milliseconds
    public static void delay(long ms)
    {
        try {
            TimeUnit.MILLISECONDS.sleep(ms);
        }
        catch(InterruptedException e) {
            
        }
    }
    
    //sets the color of the built in leds 
    public void light(int color) {
        control_Hub.setConstant(color);
        expansion_Hub_2.setConstant(color);
    }
    
    void webcamSetup() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        
        webcam.setPipeline(new octoColor());

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                
            }
        });
    }
    
    void motorSetup() {
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftBack.setDirection(DcMotor.Direction.REVERSE);
        
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setDirection(DcMotor.Direction.REVERSE);
        
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setDirection(DcMotor.Direction.REVERSE);
        
        roller = hardwareMap.get(Servo.class, "roller");
    }
    
    public void gyroSetup() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        
        parameters.mode             = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit        = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled   = false;

        c_imu = hardwareMap.get(BNO055IMU.class, "imu");
        e_imu = hardwareMap.get(BNO055IMU.class, "imu2");

        c_imu.initialize(parameters);
        e_imu.initialize(parameters);
    }
    
    public void miscSetup() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        
        front = hardwareMap.get(DistanceSensor.class, "front");
        distL = hardwareMap.get(DistanceSensor.class, "distL");
        distR = hardwareMap.get(DistanceSensor.class, "distR");
        //claw = hardwareMap.get(Servo.class, "claw");
    }
    
    // todo: write your code here
}

class octoColor extends OpenCvPipeline {
    public static int maxColor = 0;
    
    Mat blurredImage = new Mat();
    Mat hsvImage = new Mat();
    
    Mat thresh = new Mat();
    
    Mat output = new Mat();
    
    Mat dilated = new Mat();
    Mat hierarchey = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();
    
    private double colorProcess(Mat input, Scalar minColor, Scalar maxColor, Scalar color) {
        contours.clear();

        output = input;
        
        Imgproc.blur(input, blurredImage, new Size(3, 3));
        Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);
        
        Core.inRange(hsvImage, minColor, maxColor, thresh);     
        
        //return colorRange;
        
        //dilate the image, to make it better for the next step
        //make the blobs bigger
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.dilate(thresh, dilated, kernel);
        
        //find the contours of the image
        Imgproc.findContours(dilated, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        
        Imgproc.drawContours(input, contours, -1, color, 1);
        
        double maxVal = 0;
        int maxValIdx = 0;
        if (!contours.isEmpty()) {
            for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++)
            {
                double contourArea = Imgproc.contourArea(contours.get(contourIdx));
                if (maxVal < contourArea)
                {
                    maxVal = contourArea;
                    maxValIdx = contourIdx;
                }
            }
        } 
        
        return maxVal;
    }
    
    public static double purple;
    public static double green;
    public static double orange;
    
    @Override 
    public Mat processFrame(Mat input)
    {
        purple = colorProcess(input, new Scalar(0, 175, 10), new Scalar(30, 255, 75), new Scalar(0, 0, 255));
        orange = colorProcess(input, new Scalar(94, 181, 88), new Scalar(110, 255, 214), new Scalar(255, 0, 0)) - 300;
        green = colorProcess(input, new Scalar(40, 140, 40), new Scalar(80, 220, 110), new Scalar(0, 255, 0));
        
        if (orange > purple && orange > green) {
            maxColor = 1;
        } 
        if (purple > orange && purple > green) {
            maxColor = 2;
        }
        if (green > purple && green > orange) {
            maxColor = 3;
        }
        
        return input;
    }
}
