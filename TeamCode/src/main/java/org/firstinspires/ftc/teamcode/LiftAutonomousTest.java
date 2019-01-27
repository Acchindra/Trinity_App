package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
/**
 * Created by Manjesh on 12/4/2018.
 */
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
    final int value = 7250;

    public DcMotor liftMotor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (opModeIsActive()) {
            telemetry.addData("Status", "Good Luck Drivers");

            int position = liftMotor.getCurrentPosition();
            telemetry.addData("Encoder Position", position);

            Lift(DRIVE_SPEED, value);



            telemetry.addData("Path", "Complete");
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

}