package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;

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

public class Arepikov_Mechanum_Auton extends LinearOpMode{
    OpenCvWebcam webcam;
    
    public static int x = 0;
    public static int y = 0;
    
    private Blinker         control_Hub, expansion_Hub_2;
    private BNO055IMU       c_imu, e_imu;
    private DcMotor intake, leftBack, leftFront, rightBack, rightFront, lift, lift2, carousel1, carousel2;
    private TouchSensor     touch, mag;
    private DistanceSensor  ultra;
    private Servo claw, flipperLeft, flipperRight;
    
    private double[]        motor = {0, 0, 0, 0, 0, 0, 0, 0};
    
    //angle is the angle read directly from the sensors
    //actual angle has the offset applied
    private double          actualAngle, angle;
    private double          offset;
    
    int liftPosition = 0;
    int liftPosition2 = 0;

    //setup land
    
    public void motorSetup()
    {
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightBack.setDirection(DcMotor.Direction.REVERSE);
        
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake.setDirection(DcMotor.Direction.REVERSE);
        
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setPower(0);
        lift.setTargetPosition(lift.getCurrentPosition());
        liftPosition = lift.getCurrentPosition();
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift2.setPower(0);
        lift2.setTargetPosition(lift2.getCurrentPosition());
        liftPosition2 = lift2.getCurrentPosition();
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lift.setDirection(DcMotor.Direction.REVERSE);
        
        //carousel1 = hardwareMap.get(DcMotor.class, "carRight");
        //carousel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //carousel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //carousel1.setDirection(DcMotor.Direction.REVERSE);
        
        //Only using one carousel for now. hardware connects carLeft
        carousel2 = hardwareMap.get(DcMotor.class, "carLeft");
        carousel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //carousel2.setDirection(DcMotor.Direction.REVERSE);
    }
    
    public void gyroSetup()
    {
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
    
    public void sensorSetup()
    {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        
        //mag = hardwareMap.get(TouchSensor.class, "limit");
        //touch = hardwareMap.get(TouchSensor.class, "touchy");
        //ultra = hardwareMap.get(DistanceSensor.class, "distance");
        claw = hardwareMap.get(Servo.class, "claw");
        flipperLeft = hardwareMap.get(Servo.class, "flipperLeft");
        flipperRight = hardwareMap.get(Servo.class, "flipperRight");
    }
    
    public void webcamSetup() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new colorPipe());

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
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }
    
    //::angle land
    
    public void getAngle()
    {
        Orientation c_angles = c_imu.getAngularOrientation(
        AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        Orientation e_angles = e_imu.getAngularOrientation(
        AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        angle = (c_angles.firstAngle + e_angles.firstAngle)/2;
        actualAngle = angle + offset;
    }
    
    public void offset()
    {
        offset = -angle;
    }
    
    //::other internal functions
    public void setPower()
    {
        telemetry.addLine("HI");
        rightBack.setPower(motor[0]/100);
        leftBack.setPower(motor[1]/100);
        leftFront.setPower(motor[2]/100);
        rightFront.setPower(motor[3]/100);
        
        intake.setPower(motor[4]/100);
        carousel2.setPower(motor[6]/100);
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
    public void light(int color)
    {
        control_Hub.setConstant(color);
        expansion_Hub_2.setConstant(color);
        
    }
    
    //gets the distance (in millimeters) from the distance sensor ultra
    public double getDistance()
    {
        return ultra.getDistance(DistanceUnit.MM);
    }
    
    //is the touch sensor touch pressed?
    public boolean getTouch()
    {
        return touch.isPressed();
    }
    
    //is the magnetic sensor pressed?
    public boolean getMag()
    {
        return mag.isPressed();
    }
    
    //motor access functions
    //use these!
    /* notes on all these functions
    every instance of power as a parameter expects inputs in the range of -100, 100.
    every instance of angle as a parameter expects inputs in the range of -180, 180.
    every instance of milliseconds as a parameter expects inputs in the range of 0, ∞
    every function refixed with go_ is a function that turns on the motors and does not turn them off
        make sure to call stopMotors or something!
    */
    
    //move at the angle specified
    //0 is forward relative to the robot
    private void goAngle(double power, double angle, int milliseconds)
    {
        go_GoAngle(power, angle);
        delay(milliseconds);
        stopMotors();
    }
    
    private void go_GoAngle(double power, double angle)
    {
        double robotAngle = (angle+90)*(Math.PI/180);
        
        motor[0] = power * Math.sin(robotAngle + (Math.PI/4));
        motor[1] = power * Math.sin(robotAngle - (Math.PI/4));
        motor[2] = power * Math.sin(robotAngle + (Math.PI/4));
        motor[3] = power * Math.sin(robotAngle - (Math.PI/4));
        
        setPower();
    }
    
    //set the raw power of the motors
    //power is a list of parameters. 
    //ex: setRawPower(1000, 100, 100, 50, 75)
    //sets motor 0 to 100, motor 1 to 100, motor 2 to 50, and motor 3 to 75
    //right back, left back, left front, right back
    private void rawPower(int milliseconds, double... power)
    {
        for (int i = 0; i<power.length && i<=4; i++)
        {
            motor[i] = power[i];
        }
        
        setPower();
        delay(milliseconds);
        stopMotors();
    }
    
    private void go_RawPower(double... power)
    {
        for (int i = 0; i<power.length && i<=4; i++)
        {
            motor[i] = power[i];
        }
        
        setPower();
    }
    
    //stops the motors
    private void stopMotors()
    {
        motor[0] = 0;
        motor[1] = 0;
        motor[2] = 0;
        motor[3] = 0;
        motor[6] = 0;
        motor[4] = 0;
        setPower();
    }
    
    //turns the robot for a certain amount of time
    //positive power turns left, negative turns right
    private void turnTime(double power, long time) {
        motor[0] = power;
        motor[3] = power;
        
        motor[1] = -power;
        motor[2] = -power;
        
        setPower();
        delay(time);
        stopMotors();
    }
    
    //turns the robot
    private void turn(double power, double turnAngle)
    {
        power = Math.abs(power);
        getAngle();
        double deltaAngle = actualAngle+turnAngle;
        while (Math.abs(deltaAngle) > 5)
        {
            opModeIsActive();
            if (deltaAngle > 0)
            {
                motor[0] = power;
                motor[1] = -power;
                motor[2] = -power;
                motor[3] = power;
            } else {
                motor[0] = -power;
                motor[1] = power;
                motor[2] = power;
                motor[3] = -power;
            }
            getAngle();
            setPower();
            deltaAngle = turnAngle-actualAngle;
        }
    }
    
    /* the same as the above function, but with an optional accuracy parameter
    // accuracy is how many degrees +- the correct is acceptable
    // considering the above function works at all powers, 
    // this would mainly be used to dial in the accuracy tighter. 
    // if you're not careful the robot will skip over the accuracy and will spin forever
    // use at your own risk.
    */
    private void turn(double power, double accuracy, double turnAngle)
    {
        power = Math.abs(power);
        getAngle();
        double deltaAngle = actualAngle+turnAngle;
        while (Math.abs(deltaAngle) > accuracy)
        {
            opModeIsActive();
            if (deltaAngle > 0)
            {
                motor[0] = power;
                motor[1] = -power;
                motor[2] = -power;
                motor[3] = power;
            } else {
                motor[0] = -power;
                motor[1] = power;
                motor[2] = power;
                motor[3] = -power;
            }
            getAngle();
            setPower();
            deltaAngle = turnAngle-actualAngle;
        }
    }
    
    /* in order
    rightBack = 0
    leftBack = 1
    leftFront = 2
    rightFront = 3
    
    intake = 4
    carousel on the right = 5
    carousel on the left = 6
    */
    private void turnSingleMotor(int motorNumber, double power, int milliseconds) 
    {
        go_TurnSingleMotor(motorNumber, power);
        delay(milliseconds);
        stopMotors();
    }
    
    private void go_TurnSingleMotor(int motorNumber, double power)
    {
        motor[motorNumber] = power;
        setPower();
    }
    
    //move the lift by changeAmount
    //290-ish movement is roughly one rotation
    //need to check how much is level. 
    //negative values to go down. 
    void changeLiftPosition(int changeAmount) {
        liftPosition += changeAmount;
    }
    
    //identify barcode
    public int barcode(float line1, float line2){
        if (x <= line1) {
            return 1;
        } else if (x <= line2) {
            return 2;
        } else {
            return 3;
        }
    }
    
    @Override
    public void runOpMode() {
        opModeIsActive();
        //put setup stuff here
        motorSetup();
        gyroSetup();
        sensorSetup();
        webcamSetup();
        
        getAngle();
        
        telemetry.addLine("Ready!");
        telemetry.addData("angle", actualAngle);
        telemetry.update();
        //servo(0);
        
        waitForStart();
        //code goes here
        telemetry.addData("x of webcam", x);
        telemetry.update();
        
        //code for going to shipping hub
        
        flipperLeft.setPosition(0);
        flipperRight.setPosition(1);
        delay(1250);
        flipperLeft.setPosition(0.5);
        flipperRight.setPosition(0.5);
        
        int elementpos = barcode(100, 250); //values to be substituted
        telemetry.addData("team element position", elementpos);
        telemetry.update();
        
        //code for lift
        float[] liftlevels = {0, -225, -1100}; //values to be substituted
        //lift.setPower(1);
        //lift2.setPower(1);
        liftPosition += liftlevels[elementpos - 1]; 
        liftPosition2 -= liftlevels[elementpos- 1];
        lift.setTargetPosition(liftPosition);
        lift2.setTargetPosition(liftPosition2);
        
        
        goAngle(100, 20, 1000);
        turnTime(50, 600);
        goAngle(50, 0, 200);
        turnSingleMotor(6, 50, 3250);
        //
        goAngle(100, 140, 900);
        //code ends here
        while(opModeIsActive())
        {
            stopMotors();
            delay(30);
            
        }
    }
    
    class colorPipe extends OpenCvPipeline
    {
        Mat blurredImage = new Mat();
        Mat hsvImage = new Mat();
        
        Mat thresh = new Mat();
        
        Mat output = new Mat();
        
        Mat dilated = new Mat();
        Mat hierarchey = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        
        @Override
        public Mat processFrame(Mat input)
        {
            contours.clear();
            
            output = input;
            
            Imgproc.blur(input, blurredImage, new Size(3, 3));
            Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);
            
            Core.inRange(hsvImage, new Scalar(20, 75, 76), new Scalar(49, 184, 253), thresh);     
            
            //return colorRange;
            
            //dilate the image, to make it better for the next step
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.dilate(thresh, dilated, kernel);
            
            //find the contours
            Imgproc.findContours(dilated, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            Imgproc.drawContours(input, contours, -1, new Scalar(0,255,0), 1);
            
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
            
            //make sure the contour is big enough. if not, put some text on screen and return.
            if (maxVal < 100) 
            {
                Imgproc.putText(output, "no target", new Point(10, 20), 0, 1, new Scalar(0, 0, 255));
                return output;
            }
            
            Imgproc.drawContours(input, contours, maxValIdx, new Scalar(255,0,0), 2);
            
            Moments mu = Imgproc.moments(contours.get(maxValIdx));
            Arepikov_Mechanum_Auton.x = (int) (mu.get_m10() / mu.get_m00());
            Arepikov_Mechanum_Auton.y = (int) (mu.get_m01() / mu.get_m00());
            
            //draw a circle at the center of the contour, write where it is, and exit
            Imgproc.circle(output, new Point(x, y), 3, new Scalar(0,255,255));
            Imgproc.putText(output, "target at x = " + Arepikov_Mechanum_Auton.x, new Point(10, 20), 1, 1, new Scalar(0, 0, 0));
            
            
            return output;
            
        }
    }
}

class color
{
    //
    static int green =      0x00ff00;
    
    //
    static int lightBlue =  0x00ffff;
    
    //
    static int darkBlue =   0x0000ff;
    
    //
    static int red =        0xff0000;
    
    //
    static int yellow =     0xffff00;
    
    //
    static int black =      0x000000;
    
    //
    static int white =      0xffffff;

    //static int lightGreen = 0xbada55;
    //static int darkGreen = 0x65535;
    //static int darkRed = 0x800000;
    //static int pink = 0xff80ed;
    //static int uglyYellow = 0x996515
}