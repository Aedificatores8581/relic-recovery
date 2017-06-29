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
@Autonomous(name = "LennyBot: Auto Test Function", group = "( \u0361\u00B0 \u035C\u0296 \u0361\u00B0 )")
@Disabled
public class LennyBotTest extends OpMode
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

    //--------------------------------------------------------------------------
    //
    // PootisBotManual
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------
    public LennyBotTest()
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
    }

    void runToPosition(int position) {
        boolean done = false;
        int targetPos = position + left.getCurrentPosition();
        int direction = position/Math.abs(position);

        left.setPower(direction);
        right.setPower(direction);

        while (!done) {
            if (direction * (left.getCurrentPosition() - targetPos) >= 0) {
                left.setPower(0.0f);
                right.setPower(0.0f);
                done = true;
            }
        }
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

        switch (state) {
            case 0:
                runToPosition(10000); state = 1; break;
            case 1:
                runToPosition(-5000); state = 2; break;
            case 2:
                runToPosition(-10000); state = 3; break;
            case 3:
                runToPosition(5000); state = 4; break;
        }

        telemetry.addData("Left Motor Pos.", left.getCurrentPosition());
        telemetry.addData("Right Motor Pos.", right.getCurrentPosition());

    } // PootisBotManual::loop

} // PootisBotManual
