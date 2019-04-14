/*

package org.firstinspires.ftc.teamcode;

import  com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Test Tele-Op")

public class LiftTeleOpTest extends LinearOpMode
{
    public DcMotor extendMotor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        extendMotor = hardwareMap.dcMotor.get("extendMotor");
        extendMotor.setDirection(DcMotor.Direction.FORWARD);
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive())
        {
            double arm_out;

            //Gamepad 2 Portion
            //-------------------------------------------------------------------------

            arm_out = gamepad2.left_stick_y;

            arm_out = Range.clip(arm_out, -1,1);

            arm_out = (float) scaleInput(arm_out);
            extendMotor.setPower(arm_out);

            telemetry.addData("Current Value", extendMotor.getCurrentPosition());
            telemetry.update();
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

*/
