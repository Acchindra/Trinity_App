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
/**
 * Created by Manjesh on 12/4/2018.
 */
@Autonomous(name="Terry Wang Flex Depot",group = "British Columbia Competition")
public class LeftTrinityAutonomous extends LinearOpMode {

    //Declare all variables
    final float CIRCUMFERENCE = (float)(3.93701 * Math.PI);
    final int ENCODERTICKS = 723;
    final double GEARRATIO = 0.34;
    final double COUNTS_PER_INCH = (ENCODERTICKS * GEARRATIO) / (CIRCUMFERENCE);
    final double STRAFE_SPEED = 0.4;
    final double DRIVE_SPEED             = 1;     // Nominal speed for better accuracy.
    final double TURN_SPEED              = 0.5;
    final double HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    final double P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    final double P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    final int value = 7600;

    //Declare all motors
    public DcMotor rightMotorFront;
    public DcMotor leftMotorFront;
    public DcMotor rightMotorBack;
    public DcMotor leftMotorBack;
    public DcMotor liftMotor;
    public Servo flickServo;;
    public ModernRoboticsI2cGyro gyro;
    private GoldAlignDetector detector;

    @Override
    public void runOpMode() throws InterruptedException
    {
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        liftMotor = hardwareMap.dcMotor.get("linearactuatorMotor");
        flickServo = hardwareMap.servo.get("depositServo");
        leftMotorBack.setDirection(DcMotor.Direction.REVERSE);
        leftMotorFront.setDirection(DcMotor.Direction.REVERSE);
        rightMotorFront.setDirection(DcMotor.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.get("gyro");

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (opModeIsActive()) {
            while (!isStarted()) {
                telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
                telemetry.update();
            }
            telemetry.addData("Status", "Good Luck Drivers");

            detector = new GoldAlignDetector();
            detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
            detector.useDefaults();

            // Optional Tuning
            detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
            detector.downscale = 0.4; // How much to downscale the input frames

            detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
            detector.maxAreaScorer.weight = 0.005;

            detector.ratioScorer.weight = 5;
            detector.ratioScorer.perfectRatio = 1.0;

            gyro.resetZAxisIntegrator();

            detector.enable();
            sleep(1500);

            //Center
            if (detector.getXPosition() > 180 && detector.getXPosition() < 420) {

                /*detector.disable();
                StopDriving();
                Lift(1.0, value);
                gyroTurn(TURN_SPEED,0);
                gyroHold(TURN_SPEED,0,0.5);
                gyroDrive(DRIVE_SPEED, 4.5, 0);
                strafe(STRAFE_SPEED, .6, 165);
                sleep(1000);
                gyroTurn(TURN_SPEED, 45);
                flickServo.setPosition(Servo.MIN_POSITION);
                sleep(1000);
                gyroDrive(DRIVE_SPEED, -30, 45);
*/


                //Tank Drive (Single Sampling)

                detector.disable();
                StopDriving();
                Lift(DRIVE_SPEED, value);
                gyroTurn(TURN_SPEED,0);
                gyroHold(TURN_SPEED,0,0.5);
                gyroDrive(DRIVE_SPEED, 15, 0);
                gyroTurn(TURN_SPEED, 90);
                gyroHold(TURN_SPEED,90,1);
                gyroDrive(DRIVE_SPEED, 250, 90 );
                flickServo.setPosition(Servo.MIN_POSITION);
                sleep(1000);
                gyroTurn(TURN_SPEED, 40);
                gyroHold(TURN_SPEED, 40, 1);
                gyroDrive(DRIVE_SPEED, -300, 41);

                //Mechanum Drive (Double Sampling)

                /*detector.disable();
                StopDriving();
                Lift(DRIVE_SPEED, value);
                gyroTurn(TURN_SPEED, 0);
                gyroHold(TURN_SPEED,0,0.5);
                gyroDrive(DRIVE_SPEED, 3, 0);
                strafe(STRAFE_SPEED, 2, 190);
                gyroTurn(TURN_SPEED, 90);
                gyroHold(TURN_SPEED,90,5);
                flickServo.setPosition(Servo.MIN_POSITION);
                sleep(1000);
                gyroDrive(DRIVE_SPEED, 60, 90);
                strafe(STRAFE_SPEED, 5, 225);
                gyroDrive(DRIVE_SPEED, -10, -90);*/
            }

            //Right
            else if (detector.getXPosition() > 420){

                /*detector.disable();
                StopDriving();
                Lift(1.0, value);
                gyroTurn(TURN_SPEED,0);
                gyroHold(TURN_SPEED,0,0.5);
                gyroDrive(DRIVE_SPEED, 4.5, 0);
                strafe(STRAFE_SPEED, .05, 180);
                gyroDrive(-DRIVE_SPEED, 4, 0);
                strafe(STRAFE_SPEED, .5, 135);
                gyroTurn(TURN_SPEED, 40);
                gyroHold(TURN_SPEED,0,0.5);
                strafe(STRAFE_SPEED, .5, 180);
                flickServo.setPosition(Servo.MIN_POSITION);
                sleep(1000);
                strafe(STRAFE_SPEED, 1, 270);
*/
                detector.disable();
                StopDriving();
                Lift(DRIVE_SPEED, value);
                gyroTurn(TURN_SPEED,0);
                gyroHold(TURN_SPEED,0,0.5);
                gyroDrive(DRIVE_SPEED, 15, 0);
                gyroTurn(TURN_SPEED, 55);
                gyroHold(TURN_SPEED, 55, 0.5);
                gyroDrive(DRIVE_SPEED, 185, 55);
                gyroTurn(TURN_SPEED, 125);
                gyroHold(TURN_SPEED, 125, 0.5);
                gyroDrive(DRIVE_SPEED, 170, 120);
                gyroTurn(TURN_SPEED,45);
                gyroHold(TURN_SPEED,45,0.5);
                flickServo.setPosition(Servo.MIN_POSITION);
                sleep(1000);
                gyroDrive(DRIVE_SPEED, -312, 41);
            }

            //Left
            else if (detector.getXPosition() < 180) {

                /*detector.disable();
                StopDriving();
                Lift(1.0, value);
                gyroTurn(TURN_SPEED,0);
                gyroHold(TURN_SPEED,0,0.5);
                gyroDrive(DRIVE_SPEED, 4.5, 0);
                strafe(STRAFE_SPEED, .05, 180);
                gyroDrive(-DRIVE_SPEED, 4, 0);
                strafe(STRAFE_SPEED, .5, 240);
                gyroTurn(TURN_SPEED, 40);
                gyroHold(TURN_SPEED,0,0.5);
                strafe(STRAFE_SPEED, .2, 90);
                flickServo.setPosition(Servo.MIN_POSITION);
                sleep(1000);
                strafe(STRAFE_SPEED, 1, 270);*/

                detector.disable();
                StopDriving();
                Lift(DRIVE_SPEED, value);
                gyroTurn(TURN_SPEED,0);
                gyroHold(TURN_SPEED,0,0.5);
                gyroDrive(DRIVE_SPEED, 15, 0);
                gyroTurn(TURN_SPEED, 115);
                gyroHold(TURN_SPEED, 115, 0.5);
                gyroDrive(DRIVE_SPEED, 188, 115);
                gyroTurn(TURN_SPEED, 50);
                gyroHold(TURN_SPEED, 50, 0.5);
                gyroDrive(DRIVE_SPEED, 150, 50);
                flickServo.setPosition(Servo.MIN_POSITION);
                sleep(1000);
                gyroDrive(DRIVE_SPEED, -340, 41);
            }

            telemetry.addData("Path", "Complete");
            telemetry.update();

            StopDriving();
        }

    }

    public void StopDriving ()
    {
        leftMotorFront.setPower(0);
        rightMotorBack.setPower(0);
        rightMotorFront.setPower(0);
        leftMotorBack.setPower(0);
        liftMotor.setPower(0);
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

    public void gyroDrive ( double speed, double distance, double angle) {

        int     newLeftTargetFront;
        int     newLeftTargetBack;
        int     newRightTargetFront;
        int     newRightTargetBack;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTargetFront = leftMotorFront.getCurrentPosition() + moveCounts;
            newLeftTargetBack = leftMotorBack.getCurrentPosition() + moveCounts;
            newRightTargetFront = rightMotorFront.getCurrentPosition() + moveCounts;
            newRightTargetBack = rightMotorBack.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            rightMotorFront.setTargetPosition(newRightTargetFront);
            rightMotorBack.setTargetPosition(newRightTargetBack);
            leftMotorFront.setTargetPosition(newLeftTargetFront);
            leftMotorBack.setTargetPosition(newLeftTargetBack);

            rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            rightMotorFront.setPower(speed);
            rightMotorBack.setPower(speed);
            leftMotorFront.setPower(speed);
            leftMotorBack.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (rightMotorFront.isBusy() && rightMotorBack.isBusy() && leftMotorFront.isBusy() && leftMotorBack.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftMotorFront.setPower(leftSpeed/2);
                leftMotorBack.setPower(leftSpeed/2);
                rightMotorFront.setPower(rightSpeed/2);
                rightMotorBack.setPower(rightSpeed/2);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newRightTargetFront, newRightTargetBack, newLeftTargetFront, newLeftTargetBack);
                telemetry.addData("Actual",  "%7d:%7d",      leftMotorFront.getCurrentPosition(), leftMotorBack.getCurrentPosition(), rightMotorFront.getCurrentPosition(), rightMotorBack.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            leftMotorFront.setPower(0);
            leftMotorBack.setPower(0);
            rightMotorFront.setPower(0);
            rightMotorBack.setPower(0);

            // Turn off RUN_TO_POSITION
            rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        leftMotorFront.setPower(0);
        leftMotorBack.setPower(0);
        rightMotorFront.setPower(0);
        rightMotorBack.setPower(0);
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftMotorFront.setPower(leftSpeed);
        leftMotorBack.setPower(leftSpeed);
        rightMotorFront.setPower(rightSpeed);
        rightMotorBack.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}