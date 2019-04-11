package org.firstinspires.ftc.teamcode;

import  com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Manjesh on 12/4/2018.
 */

@TeleOp(name = "Tele-Op")
public class Teleop extends LinearOpMode
{
    //Motor Count -- 8 / 8 (Max Usage)
    public DcMotor rightMotorFront;
    public DcMotor leftMotorFront;
    public DcMotor rightMotorBack;
    public DcMotor leftMotorBack;
    public DcMotor liftMotor;
    public DcMotor extendMotor;
    public DcMotor linearactuatorMotor;

    //Servo Count -- 4 / 12 (8 left)
    public Servo rotateServoOne;//black box
    //public Servo rotateServoTwo;//black box
    public Servo depositServo;
    public CRServo vacuumServo;//collection mechanism

    @Override
    public void runOpMode() throws InterruptedException
    {
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        extendMotor = hardwareMap.dcMotor.get("extendMotor");
        linearactuatorMotor = hardwareMap.dcMotor.get("linearactuatorMotor");
        rotateServoOne = hardwareMap.servo.get("rotateServoOne");
        depositServo = hardwareMap.servo.get("depositServo");
        //rotateServoTwo = hardwareMap.servo.get("rotateServoTwo");
        vacuumServo = hardwareMap.crservo.get("vacuumServo");
        leftMotorBack.setDirection(DcMotor.Direction.FORWARD);
        leftMotorFront.setDirection(DcMotor.Direction.FORWARD);
        rightMotorFront.setDirection(DcMotor.Direction.REVERSE);
        rightMotorBack.setDirection(DcMotor.Direction.REVERSE);
        linearactuatorMotor.setDirection(DcMotor.Direction.FORWARD);
        extendMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);


        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearactuatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearactuatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        while (opModeIsActive())
        {
            //Introduce variables
            double drive;   // Power for forward and back motion
            double strafe;  // Power for left and right motion
            double rotate;  // Power for rotating the robot
            double GoUPandDOWN;
            double arm_out;
            double arm_up;

            //Gamepad 1 Portion
            //-------------------------------------------------------------------------
            drive = -gamepad1.left_stick_y;  // Negative because the gamepad is weird
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;


            //Set the values for the drive to be only -1 <-> 1
            drive = Range.clip(drive, -1, 1);
            strafe = Range.clip(strafe, -1, 1);
            rotate = Range.clip(rotate, -1, 1);

            //Set the variables drive component to work with our custom method
            drive = (float) scaleInput(drive);
            strafe = (float) scaleInput(strafe);
            rotate = (float) scaleInput(rotate);

            if (gamepad1.left_trigger > 0.25)
            {
                drive /= 3;
                strafe /= 3;
                rotate /= 3;
            }

            if(gamepad1.right_trigger > 0.25){
                leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            //Set the power for the wheels
            leftMotorBack.setPower(drive - strafe + rotate);
            leftMotorFront.setPower(drive + strafe + rotate);
            rightMotorBack.setPower(drive + strafe - rotate);
            rightMotorFront.setPower(drive - strafe - rotate);

            //Linear Actuator Control (Motor)
            GoUPandDOWN = gamepad2.left_trigger;

            if (gamepad1.right_bumper)
            {
                GoUPandDOWN = 1.0;
            }

            if (gamepad1.left_bumper)
            {
                GoUPandDOWN = -1.0;
            }

            linearactuatorMotor.setPower(GoUPandDOWN);

            //Gamepad 2 Portion
            //-------------------------------------------------------------------------

            //Linear Slide Up Control (Motor)
            arm_up = gamepad2.right_stick_y;

            arm_up = Range.clip(arm_up, -0.5,0.5);

            arm_up = (float) scaleInput(arm_up);

            liftMotor.setPower(arm_up);

            //Linear Slide Out Control (Motor)
            arm_out = gamepad2.left_stick_y;

            arm_out = Range.clip(arm_out, -1,1);

            arm_out = (float) scaleInput(arm_out);
            extendMotor.setPower(arm_out);

            //Vacuum Control (Servo)
            if (gamepad2.a)
            {
                vacuumServo.setPower(-1.0);
            }

            if (gamepad2.b)
            {
                vacuumServo.setPower(1.0);
            }

            if (gamepad2.x)
            {
                vacuumServo.setPower(0);
            }

            //Arm Rotate Control (Servo)
            if (gamepad2.dpad_down)
            {
                rotateServoOne.setPosition(Servo.MAX_POSITION);
                //rotateServoTwo.setPosition(Servo.MIN_POSITION);
            }

            if (gamepad2.dpad_up)
            {
                //rotateServoTwo.setPosition(Servo.MAX_POSITION);
                rotateServoOne.setPosition(Servo.MIN_POSITION);
            }

            if (gamepad2.right_bumper)
            {
                //rotateServoTwo.setPosition(0.5);
                rotateServoOne.setPosition(0.5);
            }


            if (gamepad2.right_trigger > 0.25){
                while (!gamepad2.y){
                    move(extendMotor,0.5,0);
                    rotateServoOne.setPosition(Servo.MAX_POSITION);
                    //rotateServoTwo.setPosition(Servo.MIN_POSITION);
                }
            }

            if (gamepad2.left_trigger > 0.25){
                while (!gamepad2.y){
                    move(liftMotor,1, 3000);
                    sleep(1000);
                    move(liftMotor,1, 0);
                }
            }

            //Fail safe actions
            //-------------------------------------------------------------------------
            depositServo.setPosition(Servo.MIN_POSITION);
        }
    }

    public void move (DcMotor motor, double power, int distance)
    {
        int moveNumber;
        int move;

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
//            moveNumber = (int)(distance);
//            move = motor.getCurrentPosition() + moveNumber;

            // Set Target and Turn On RUN_TO_POSITION
            motor.setTargetPosition(distance);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion
            power = Range.clip(Math.abs(power), 0.0, 1.0);
            motor.setPower(power);

            while (opModeIsActive() && motor.getCurrentPosition() < motor.getTargetPosition()) {
                telemetry.addData("Current Value", motor.getCurrentPosition());
                telemetry.addData("Target Value", motor.getTargetPosition());
                telemetry.update();
                idle();
            }
        }
    }

    double scaleInput(double dVal)
    {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        }
        if (index > 16) {
            index = 16;
        }
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        return dScale;
    }
}