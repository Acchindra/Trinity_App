/*
package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
*/
/**
 * Created by Manjesh on 12/4/2018.
 *//*

@Autonomous(name="LiftTest")
public class LiftAutonomousTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //Declare all variables
    final float CIRCUMFERENCE = (float)(3.93701 * Math.PI);
    final int ENCODERTICKS = 1680;
    final double GEARRATIO = 0.67;
    final double COUNTS_PER_INCH = (ENCODERTICKS * GEARRATIO) / (CIRCUMFERENCE);
    final double DRIVE_SPEED             = 1.0;     // Nominal speed for better accuracy.
    final double TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.
    final double HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    final double P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    final double P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    final int value = 90000;

    public DcMotor liftMotor;
    public DcMotor rightMotorFront;
    public DcMotor leftMotorFront;
    public DcMotor rightMotorBack;
    public DcMotor leftMotorBack;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //liftMotor = hardwareMap.dcMotor.get("linearactuatorMotor");
        //liftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        rightMotorFront.setDirection(DcMotor.Direction.FORWARD);
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        leftMotorFront.setDirection(DcMotor.Direction.REVERSE);
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");
        rightMotorBack.setDirection(DcMotor.Direction.FORWARD);
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        leftMotorBack.setDirection(DcMotor.Direction.REVERSE);


        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

        //liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (opModeIsActive()) {
            telemetry.addData("Status", "Good Luck Drivers");

            rightMotorFront.setTargetPosition(900000);
            leftMotorFront.setTargetPosition(900000);
            rightMotorBack.setTargetPosition(900000);
            leftMotorBack.setTargetPosition(900000);


            rightMotorFront.setPower(0.4);
            leftMotorFront.setPower(0.4);
            rightMotorBack.setPower(0.4);
            leftMotorBack.setPower(0.4);

            rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (opModeIsActive() && rightMotorFront.getCurrentPosition() < rightMotorFront.getTargetPosition() && leftMotorFront.getCurrentPosition() < leftMotorFront.getTargetPosition() && rightMotorBack.getCurrentPosition() < rightMotorBack.getTargetPosition() && leftMotorBack.getCurrentPosition() < leftMotorBack.getTargetPosition())
            {
                telemetry.addData("RIGHT FRONT Encoder Position", rightMotorFront.getCurrentPosition());
                telemetry.addData("Target Value", rightMotorFront.getTargetPosition());
                telemetry.addData("LEFT FRONT Encoder Position", leftMotorFront.getCurrentPosition());
                telemetry.addData("Target Value", leftMotorFront.getTargetPosition());
                telemetry.addData("RIGHT BACK Encoder Position", rightMotorBack.getCurrentPosition());
                telemetry.addData("Target Value", rightMotorBack.getTargetPosition());
                telemetry.addData("LEFT BACK Encoder Position", leftMotorBack.getCurrentPosition());
                telemetry.addData("Target Value", leftMotorBack.getTargetPosition());
                telemetry.update();
                idle();
            }


            while (opModeIsActive() && leftMotorBack.getCurrentPosition() < leftMotorBack.getTargetPosition()) {
                telemetry.addData("LEFT BACK Encoder Position", leftMotorBack.getCurrentPosition());
                telemetry.addData("Target Value", leftMotorBack.getTargetPosition());
                telemetry.update();
                idle();
            }

            rightMotorFront.setPower(0.4);
            leftMotorFront.setPower(0.4);
            rightMotorBack.setPower(0.4);
            leftMotorBack.setPower(0.4);

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }

    }

    public void strafe(double speed, double time, double angle){
        double angleInterval = speed/45;
        double speedInterval;
        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < time)) {

            if (angle >= 0 && angle < 45){
                speedInterval = 45 - angle;
                leftMotorFront.setPower(speed);
                rightMotorBack.setPower(speed);
                rightMotorFront.setPower(-1*(speedInterval*angleInterval));
                leftMotorBack.setPower(-1*(speedInterval*angleInterval));
            } else if(angle >= 45 && angle < 90){
                speedInterval = angle - 45;
                leftMotorFront.setPower(speed);
                rightMotorBack.setPower(speed);
                rightMotorFront.setPower(speedInterval*angleInterval);
                leftMotorBack.setPower(speedInterval*angleInterval);
            } else if (angle >= 90 && angle < 135){
                speedInterval = 135 - angle;
                leftMotorFront.setPower(speedInterval*angleInterval);
                rightMotorBack.setPower(speedInterval*angleInterval);
                rightMotorFront.setPower(speed);
                leftMotorBack.setPower(speed);
            } else if (angle >= 135 && angle < 180){
                speedInterval = angle - 135;
                leftMotorFront.setPower(-1*(speedInterval*angleInterval));
                rightMotorBack.setPower(-1*(speedInterval*angleInterval));
                rightMotorFront.setPower(speed);
                leftMotorBack.setPower(speed);
            } else if (angle >= 180 && angle < 225){
                speedInterval = 225 - angle;
                leftMotorFront.setPower(-1*speed);
                rightMotorBack.setPower(-1*speed);
                rightMotorFront.setPower(speedInterval*angleInterval);
                leftMotorBack.setPower(speedInterval*angleInterval);
            } else if (angle >= 225 && angle < 270){
                speedInterval = angle - 225;
                leftMotorFront.setPower(-1*speed);
                rightMotorBack.setPower(-1*speed);
                rightMotorFront.setPower(-1*(speedInterval*angleInterval));
                leftMotorBack.setPower(-1*(speedInterval*angleInterval));
            } else if (angle >= 270 && angle < 315){
                speedInterval = 315 - angle;
                leftMotorFront.setPower(-1*(speedInterval*angleInterval));
                rightMotorBack.setPower(-1*(speedInterval*angleInterval));
                rightMotorFront.setPower(-1*speed);
                leftMotorBack.setPower(-1*speed);
            } else if (angle >= 315 && angle < 360){
                speedInterval = angle - 315;
                leftMotorFront.setPower(speedInterval*angleInterval);
                rightMotorBack.setPower(speedInterval*angleInterval);
                rightMotorFront.setPower(-1*speed);
                leftMotorBack.setPower(-1*speed);
            }
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

     public void Lift (double power, int distance)
    {
        int moveNumber;
        int move;

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            moveNumber = (int)(distance);
            move = liftMotor.getCurrentPosition() + moveNumber;

            // Set Target and Turn On RUN_TO_POSITION
            liftMotor.setTargetPosition(move);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion
            power = Range.clip(Math.abs(power), 0.0, 1.0);
            liftMotor.setPower(power);

            while (opModeIsActive() && liftMotor.getCurrentPosition() < liftMotor.getTargetPosition()) {
                telemetry.addData("Current Value", liftMotor.getCurrentPosition());
                telemetry.addData("Target Value", liftMotor.getTargetPosition());
                telemetry.update();
                idle();
            }
        }
    }

}*/
