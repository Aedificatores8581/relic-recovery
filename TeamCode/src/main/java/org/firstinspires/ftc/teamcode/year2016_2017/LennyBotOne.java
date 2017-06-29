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

import java.util.Timer;
import java.util.TimerTask;

/**
 * I made this from scratch.
 *
 * @author Pootis Man
 * @version 1.0
 */
@Autonomous(name = "LennyBot: Auto Test 1", group = "( \u0361\u00B0 \u035C\u0296 \u0361\u00B0 )")
@Disabled
public class LennyBotOne extends OpMode
{
    DcMotorController mc1;
    //DcMotorController con;
    //DcMotor motor1, motor2;
    //DcMotor motor3, motor4;
    DcMotor left, right;
    //TouchSensor touch1;
    //IrSeekerSensor irseek1;
    //OpticalDistanceSensor uss1;
    float speed;
    static final int STATUS_SPEEDING = 0;
    static final int STATUS_STAYING = 1;
    static final int STATUS_SLOWING = 2;
    static final int STATUS_FINISHED = 3;
    int status;
    Timer timer;

    //--------------------------------------------------------------------------
    //
    // PootisBotManual
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------
    public LennyBotOne()
    {
        timer = new Timer();

    } // PootisBotManual::PootisBotManual

    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("lm");
        right = hardwareMap.dcMotor.get("rm");
        mc1 = hardwareMap.dcMotorController.get("mc1");
        speed = 0.0f;
        status = 0;

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
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

        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                status = 1;

                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        status = 2;

                        timer.schedule(new TimerTask() {
                            @Override
                            public void run() {
                                status = 3;
                            }
                        }, 3000);
                    }
                }, 2000);
            }
        }, 1000);

        if (status == 0 && speed <= 1.0f) speed += 0.033f;
        if (status == 2 && speed >= 0.0f) speed -= 0.067f;

        left.setPower(speed);
        right.setPower(speed);
    } // PootisBotManual::loop

} // PootisBotManual
