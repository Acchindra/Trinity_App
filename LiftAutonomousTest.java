/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Wheel Test")
public class LiftAutonomousTest extends LinearOpMode {

    public DcMotor rightMotorFront;
    public DcMotor leftMotorFront;
    public DcMotor rightMotorBack;
    public DcMotor leftMotorBack;

    @Override
    public void runOpMode() throws InterruptedException
    {
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        rightMotorFront.setDirection(DcMotor.Direction.FORWARD);
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        leftMotorFront.setDirection(DcMotor.Direction.REVERSE);
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");
        rightMotorBack.setDirection(DcMotor.Direction.FORWARD);
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        leftMotorBack.setDirection(DcMotor.Direction.REVERSE);

        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();


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


}
*/
