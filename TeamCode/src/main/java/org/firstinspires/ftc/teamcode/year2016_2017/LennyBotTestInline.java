package org.firstinspires.ftc.teamcode.year2016_2017;

//------------------------------------------------------------------------------
//
// PootisBotManual
//

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * I made this from scratch.
 *
 * @author Pootis Man
 * @version 1.0
 */
@Autonomous(name = "LennyBot: Auto Test Inline", group = "( \u0361\u00B0 \u035C\u0296 \u0361\u00B0 )")
@Disabled
public class LennyBotTestInline extends OpMode
{
    DcMotorController mc1;
    //DcMotorController con;
    //DcMotor motor1, motor2;
    //DcMotor motor3, motor4;
    DcMotor left, right;
    //TouchSensor touch1;
    //IrSeekerSensor irseek1;
    //OpticalDistanceSensor uss1;
    int state = 0;
    final int[][] positions = {{1000, 800, 600, 2200}, {1000, 800, -600, 2200}}; // Don't set these to zero. Ever.
    //final int[][] positions = {{400, 300, 600, 200}, {400, 300, -600, 200}}; // Don't set these to zero. Ever.
    int leftTargetPos, rightTargetPos, leftDirection, rightDirection;
    int leftPosition, rightPosition;

    final double powah = 0.15;
    double correction = 1.0;
    boolean corrected;

    //--------------------------------------------------------------------------
    //
    // PootisBotManual
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------
    public LennyBotTestInline()
    {


    } // PootisBotManual::PootisBotManual

    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("lm");
        right = hardwareMap.dcMotor.get("rm");
        mc1 = hardwareMap.dcMotorController.get("mc1");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        state = 0;

        corrected = false;
    }

    //--------------------------------------------------------------------------
    //
    // loop
    //
    //-------
    // Initializes the class.
    //
    // The system calls this member repeatedly while the OpMode is running.
    //--------
    @Override public void loop ()
    {

/*

        position = positions[state];
        targetPos = position + left.getCurrentPosition();
        direction = position / Math.abs(position);
        if (left.getPower()==0) {

            left.setPower(direction);
            right.setPower(direction);
        }
        if (direction * (left.getCurrentPosition() - targetPos) >= 0) {
            left.setPower(0.0f);
            right.setPower(0.0f);
            state++;
        }

        */
        if (state % 2 == 0) {
            leftPosition = positions[0][state / 2];
            rightPosition = positions[1][state / 2];
            leftTargetPos = leftPosition + left.getCurrentPosition();
            rightTargetPos = rightPosition + right.getCurrentPosition();
            leftDirection = leftPosition / Math.abs(leftPosition);
            rightDirection = rightPosition / Math.abs(rightPosition);

            left.setPower(leftDirection * powah);
            right.setPower(rightDirection * powah * correction);

            state++;
        }

        if (state % 2 == 1) {
            if (leftDirection * (left.getCurrentPosition() - leftTargetPos) >= 0)
                if (rightDirection * (right.getCurrentPosition() - rightTargetPos) >= 0) {
                    left.setPower(0.0f);
                    right.setPower(0.0f);
                    state++;

                    if (!corrected) {
                        correction *= left.getCurrentPosition();
                        correction /= right.getCurrentPosition();
                        corrected = true;
                    }
                }
        }

        if ((state / 2) >= positions[0].length) {
            state = -1;

            left.setPower(0);
            right.setPower(0);
        }

        telemetry.addData("Left Motor Pos.", left.getCurrentPosition());
        telemetry.addData("Right Motor Pos.", right.getCurrentPosition());

        telemetry.addData("Left Motor Pow.", left.getPower());
        telemetry.addData("Right Motor Pow.", right.getPower());

    } // PootisBotManual::loop

} // PootisBotManual
