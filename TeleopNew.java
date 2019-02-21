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

@TeleOp(name = "Terry Wang Flex Tele-Op")
public class TeleopNew extends LinearOpMode
{
    //Motor Count -- 7 / 8
    public DcMotor rightMotorFront;
    public DcMotor leftMotorFront;
    public DcMotor rightMotorBack;
    public DcMotor leftMotorBack;
    public DcMotor liftMotor;
    public DcMotor extendMotor;
    public DcMotor linearactuatorMotor;

    //Servo Count -- 7 / 12
    public Servo liftrotateServo;
    public Servo depositServo;
    public Servo armrotateServo;
    public CRServo vacuumServo;

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
        liftrotateServo = hardwareMap.servo.get("liftrotateServo");
        depositServo = hardwareMap.servo.get("depositServo");
        armrotateServo = hardwareMap.servo.get("armrotateServo");
        vacuumServo = hardwareMap.crservo.get("vacuumServo");
        leftMotorBack.setDirection(DcMotor.Direction.REVERSE);
        leftMotorFront.setDirection(DcMotor.Direction.REVERSE);
        rightMotorFront.setDirection(DcMotor.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotor.Direction.FORWARD);
        linearactuatorMotor.setDirection(DcMotor.Direction.FORWARD);


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

            depositServo.setPosition(Servo.MAX_POSITION);

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

            //Gamepad 2 Portion
            //-------------------------------------------------------------------------

            //Linear Actuator Control (Motor)
            GoUPandDOWN = gamepad2.right_trigger;

            if (gamepad2.right_trigger > 0.25)
            {
                GoUPandDOWN = 1.0;
            }

            if (gamepad2.left_trigger > 0.25)
            {
                GoUPandDOWN = -1.0;
            }

            GoUPandDOWN = Range.clip(GoUPandDOWN, -1, 1);

            GoUPandDOWN = (float) scaleInput(GoUPandDOWN);

            linearactuatorMotor.setPower(GoUPandDOWN);

            //Linear Slide Up Control (Motor)
            arm_up = gamepad2.left_stick_y;

            arm_up = Range.clip(arm_up, -1,1);

            arm_up = (float) scaleInput(arm_up);

            liftMotor.setPower(arm_up);

            //Linear Slide Out Control (Motor)
            arm_out = gamepad2.right_stick_y;

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
            if (gamepad2.dpad_up)
            {
                //take current value and add 0.1 each time pressed
                double position = armrotateServo.getPosition();
                position += 0.1;
                armrotateServo.setPosition(position);
            }

            if (gamepad2.dpad_down)
            {
                //take current value and add 0.1 each time pressed
                double position = armrotateServo.getPosition();
                position -= 0.1;
                armrotateServo.setPosition(position);
            }

            if (gamepad2.dpad_left)
            {
                armrotateServo.setPosition(0.5);
            }

            //Arm Rotate Control (Servo)
            if (gamepad2.right_stick_y > 0.5)
            {
                armrotateServo.setPosition(0.0);
            }

            if (gamepad2.dpad_down)
            {
                armrotateServo.setPosition(1.0);
            }

            if (gamepad2.dpad_left)
            {
                armrotateServo.setPosition(0.5);
            }

            //Fail safe actions
            //-------------------------------------------------------------------------
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