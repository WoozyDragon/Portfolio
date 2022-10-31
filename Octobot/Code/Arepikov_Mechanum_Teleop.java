package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp

public class Arepikov_Mechanum_Teleop extends LinearOpMode{
    private DcMotor intake, leftBack, leftFront, rightBack, rightFront, lift, carousel1, carousel2, lift2;
    private Blinker control_Hub, expansion_Hub_2;
    private Servo claw, flipperRight, flipperLeft;
    private BNO055IMU c_imu, e_imu;
    
    private DistanceSensor distance;
    private TouchSensor limit;
    private TouchSensor touchy;
    
    private double[] motor = {0, 0, 0, 0, 0, 0, 0};
    
    private double angle, actualAngle, offset;
    
    private boolean headless;
    
    long headlessWait = getTime();
    
    long switchTime1 = 0;
    long switchTime2 = 0;
    long switchTime3 = 0;
    //int liftPosition = 0;
    int intakePosition = 0;
    
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
        
        /*
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setPower(1);
        intake.setTargetPosition(intake.getCurrentPosition());
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake.setDirection(DcMotor.Direction.REVERSE);
        */
        
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake.setDirection(DcMotor.Direction.REVERSE);
        
        lift = hardwareMap.get(DcMotor.class, "lift");
        //lift.setPower(0.5);
        //lift.setTargetPosition(lift.getCurrentPosition());
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lift.setDirection(DcMotor.Direction.REVERSE);
        
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lift2.setDirection(DcMotor.Direction.REVERSE);
        
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
    
    public void miscSetup()
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
    
    //angle time
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
        double bumperOffset = (1-gamepad1.right_trigger)+0.1;
        
        rightBack.setPower(motor[0] * bumperOffset);
        leftBack.setPower(motor[1] * bumperOffset);
        leftFront.setPower(motor[2] * bumperOffset);
        rightFront.setPower(motor[3] * bumperOffset);
        
        //intake is already handled in void intakePower()
        //lift.setTargetPosition(liftPosition);
        lift.setPower(motor[6]);
        lift2.setPower(-motor[6]);
        
        carousel2.setPower(motor[5]*-1);
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
    
    //milliseconds
    public static long getTime()
    {
        return (System.nanoTime()) / 1000000;
    }
    
    //sets the color of the built in leds 
    public void light(int color)
    {
        control_Hub.setConstant(color);
        expansion_Hub_2.setConstant(color);
        
    }

    @Override
    public void runOpMode() {
        motorSetup();
        gyroSetup();
        miscSetup();
        
        telemetry.addData("Status", "Initialized");
        //telemetry.addData("liftTarget", lift.getTargetPosition());
        telemetry.update();
        
        
        
        waitForStart();
        while(opModeIsActive())
        {
            //liftPosition = lift.getTargetPosition();
            intakePosition = intake.getTargetPosition();
            //telemetry.addData("liftTarget", lift.getTargetPosition());
            //telemetry.addData("LiftPos", lift.getCurrentPosition());
            getAngle();
            
            generalPower();
            
            
            if (gamepad1.b)
            {
                returnToForward();
            } else if (gamepad1.y)
            {
                offset();
            } else if (gamepad1.x)
            {
                if ((getTime()) - headlessWait >= 500)
                {
                    headless = !headless;
                    headlessWait = getTime();
                }
            } 
            
            telemetry.addData("Angle", actualAngle);
            telemetry.addData("Offset", offset);
            telemetry.update();
            
            setPower();
        }
    }
    
    // y claw alt
        // b flipper alt
        
    boolean clawOpen = false;
    boolean bigOpen = false;
    boolean flipperOpen = false;
    
    void servoPower() { 
        if (gamepad2.b && getTime() - switchTime2 > 250) {
            switchTime2 = getTime();
            clawOpen = !clawOpen;
        }
        
       if (gamepad2.y && getTime() - switchTime3 > 250) {
            switchTime3 = getTime();
            bigOpen = true;
        }
        
        
        if (!clawOpen){
           claw.setPosition(0.25); 
           bigOpen = false;
        } 
        else if (bigOpen) claw.setPosition(0.8);
        else claw.setPosition(0.6);
        telemetry.addData("claw", claw.getPosition());
        
        
        flipperLeft.setPosition((gamepad2.right_stick_y + 1)/2);
        flipperRight.setPosition((-gamepad2.right_stick_y + 1)/2);
        
        /*
        if (flipperOpen) flipper.setPosition(1);
        else flipper.setPosition(0);
        */
        telemetry.addData("flipperL", flipperLeft.getPosition());
        telemetry.addData("flipperR", flipperRight.getPosition());
        
    }
    
    
    
    //void liftPower() {
    //    int moveDist = 350;
    //    
    //    if (gamepad2.dpad_up && getTime() - switchTime1 > 1000) {
    //        liftPosition -= moveDist;
    //        switchTime1 = getTime();
    //    } else if (gamepad2.dpad_down && getTime() - switchTime1 > 1000) {
    //        liftPosition += moveDist;
    //        switchTime1 = getTime();
    //    }
    //    
    //    int stickMod = (int)(gamepad2.left_stick_y * 10 + 0.1);
    //    
    //    liftPosition += stickMod;
    //}
    void liftPower() {
        motor[6] = gamepad2.left_stick_y;
    }
    
    
    
    void intakePower() { 
        /*
        if(gamepad2.a && 
            intake.getCurrentPosition() + 200 > intake.getTargetPosition() ) {
            intake.setTargetPosition(intake.getTargetPosition() - 100);
        } else if(gamepad2.right_trigger >= 0.5 && 
            intake.getCurrentPosition() - 200 < intake.getTargetPosition() ) {
            intake.setTargetPosition(intake.getTargetPosition() + 100);
        }*/
        
        if (gamepad2.dpad_down) intake.setPower(-0.6);
        else if (gamepad2.left_trigger >= 0.25) intake.setPower(0.6);
        else intake.setPower(0);
    }

    void generalPower() {
        //0 - 3: drivetrain
        //4: intake
        //5: carousel
        
        //lift has its own variable
        //both carousels are on the same power setting
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double rightX = gamepad1.right_stick_x;
        
        if (headless)
        {
            robotAngle += actualAngle*(Math.PI/180);
            light(color.red);
        } else {
            light(color.black);
        }
        
        servoPower();
        liftPower();
        intakePower();
        
        motor[0] = r * Math.sin(robotAngle - (Math.PI/4)) + rightX;
        motor[1] = r * Math.sin(robotAngle + (Math.PI/4)) - rightX;
        motor[2] = r * Math.sin(robotAngle - (Math.PI/4)) - rightX;
        motor[3] = r * Math.sin(robotAngle + (Math.PI/4)) + rightX;
        
        motor[5] = 0;
        if (gamepad2.left_bumper) motor[5] = 0.6; //changed this on feb 14th : original value: 0.5
        if (gamepad2.right_bumper) motor[5] = -0.6;
        
        
    }
    
    private void returnToForward()
    {
        //if within 10 degrees
        if (Math.abs(actualAngle) <= 20)
        {
            while (Math.abs(actualAngle) >= 2)
            {
                light(color.lightBlue);
                opModeIsActive();
                
                if (actualAngle <= 0)
                {
                    motor[0] = -0.1;
                    motor[1] = 0.1;
                    motor[2] = 0.1;
                    motor[3] = -0.1;
                } else if (actualAngle >= 0) {
                    motor[0] = 0.1;
                    motor[1] = -0.1;
                    motor[2] = -0.1;
                    motor[3] = 0.1;
                }
    
                setPower();
                getAngle();
                telemetry.addLine("Precisely Homing...");
                telemetry.addData("Actual Angle", actualAngle);
                telemetry.update();
            }
            return;
        }
        
        //if not within 10 degrees at start
        while (Math.abs(actualAngle) >= 20)
        {
            light(color.darkBlue);;
            opModeIsActive();
            
            if (actualAngle <= 0)
            {
                motor[0] = -0.75;
                motor[1] = 0.75;
                motor[2] = 0.75;
                motor[3] = -0.75;
            } else if (actualAngle >= 0) {
                motor[0] = 0.75;
                motor[1] = -0.75;
                motor[2] = -0.75;
                motor[3] = 0.75;
            }

            setPower();
            getAngle();
            telemetry.addLine("Homing...");
            telemetry.addData("Actual Angle", actualAngle);
            telemetry.update();
        }
    }
}