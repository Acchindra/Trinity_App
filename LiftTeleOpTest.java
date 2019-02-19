package org.firstinspires.ftc.teamcode;

import  com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Manjesh on 12/4/2018.
 */

@TeleOp(name = "Test Tele-Op")
public class LiftTeleOpTest extends LinearOpMode
{
    //Motor Count -- 7 / 8
    public DcMotor liftMotor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        liftMotor = hardwareMap.dcMotor.get("liftMotor");

        waitForStart();
        while (opModeIsActive())
        {
            double lift;

            //Gamepad 2 Portion
            //-------------------------------------------------------------------------

            lift = gamepad2.right_stick_y;

            lift = Range.clip(lift, -1, 1);

            lift = (float) scaleInput(lift);

            liftMotor.setPower(lift);

            idle();
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