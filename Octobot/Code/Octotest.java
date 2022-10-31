package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class Octotest extends LinearOpMode{
    DcMotor rightBack, leftBack, leftFront, rightFront;
    DcMotor leftLift, rightLift;
    Blinker control_Hub, expansion_Hub_2;
    BNO055IMU c_imu, e_imu;
    //Servo claw;
    
    boolean clawOpen = true;
    long switchTime2;
    
    double angle, rawAngle, offset;
    
    long headlessWait;
    boolean headless;

    @Override
    public void runOpMode() {
        motorSetup();
        gyroSetup();
        miscSetup();
        
        waitForStart();
        
        while (opModeIsActive()) { 
            if (gamepad1.y) {
                offset();
            } else if (gamepad1.x) {
                if ((getTime()) - headlessWait >= 500) {
                    headless = !headless;
                    headlessWait = getTime();
                }
            } 
            
            getAngle();
            generalPower();
            //servoPower();
        }

    }
    
    void generalPower() {
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double stickAngle = Math.atan2(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
        
        final double turnPower = 0.75;
        double rightX = gamepad1.right_stick_x * turnPower;
        
        double speedOffset = (gamepad1.right_trigger)+0.50;
        if(gamepad1.right_bumper) {speedOffset = 0.2;}
        
        if (Math.abs(stickAngle - Math.PI/2) < Math.PI/12) {
            stickAngle = Math.PI/2;
        } else if (Math.abs(stickAngle + Math.PI/2) < Math.PI/12) {
            stickAngle = -Math.PI/2;
        }
        
        stickAngle += Math.PI/2;
        
        if (headless)
        {
            stickAngle += angle*(Math.PI/180);
            light(color.red);
        } else {
            light(color.black);
        }
        
        rightFront.setPower((
            r * Math.sin(stickAngle - (Math.PI/4)) - rightX ) * speedOffset);
        rightBack.setPower((
            r * Math.sin(stickAngle + (Math.PI/4)) + rightX ) * speedOffset);
        leftBack.setPower((
            r * Math.sin(stickAngle - (Math.PI/4)) + rightX ) * speedOffset);
        leftFront.setPower((
            r * Math.sin(stickAngle + (Math.PI/4)) - rightX ) * speedOffset);
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
    
    /*
    void servoPower() { 
        if (gamepad2.b && getTime() - switchTime2 > 250) {
            switchTime2 = getTime();
            clawOpen = !clawOpen;
        }
        
        if (clawOpen){
           claw.setPosition(0.25); 
        } else claw.setPosition(0.725);
    }*/
    
    //set a new "0" heading for the robot
    public void offset() {
        offset = -rawAngle;
    }
    
    //in milliseconds
    public static long getTime() {
        return (System.nanoTime()) / 1000000;
    }
    
    //sets the color of the built in leds 
    public void light(int color) {
        control_Hub.setConstant(color);
        expansion_Hub_2.setConstant(color);
    }
    
    
    void motorSetup() {
        rightBack = hardwareMap.get(DcMotor.class, "rightback");
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        
        leftBack = hardwareMap.get(DcMotor.class, "leftback");
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftBack.setDirection(DcMotor.Direction.REVERSE);
        
        leftFront = hardwareMap.get(DcMotor.class, "leftfront");
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        
        rightFront = hardwareMap.get(DcMotor.class, "rightfront");
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        
        rightLift = hardwareMap.get(DcMotor.class, "rightlift");
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightLift.setDirection(DcMotor.Direction.REVERSE);
        
        leftLift = hardwareMap.get(DcMotor.class, "leftlift");
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftLift.setDirection(DcMotor.Direction.REVERSE);
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
        //claw = hardwareMap.get(Servo.class, "claw");
    }
}
