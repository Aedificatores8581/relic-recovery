package org.firstinspires.ftc.teamcode.year2016_2017;

//------------------------------------------------------------------------------
//
// PootisBotManual
//

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * I made this from scratch.
 *
 * @author Pootis Man
 * @version 1.0
 */
@TeleOp(name = "LennyBot: Ultrasonic Arrays", group = "( \u0361\u00B0 \u035C\u0296 \u0361\u00B0 )")
@Disabled
public class LennyBotUssArrayTest extends OpMode
{
    //DcMotorController con;
    //DcMotor motor1, motor2;
    //DcMotor motor3, motor4;
    //TouchSensor touch1;
    //IrSeekerSensor irseek1;
    //OpticalDistanceSensor uss1;
    UltrasonicSensor uss1;
    UltrasonicSensor uss2;
    //final double powerMult = -1.0;
    double[][] ussV;
    int count;
    boolean prevA;



    //boolean toggleLed;


    //--------------------------------------------------------------------------
    //
    // PootisBotManual
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------
    public LennyBotUssArrayTest()
    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PootisBotManual::PootisBotManual

    @Override
    public void init() {
        uss1 = hardwareMap.ultrasonicSensor.get("uss1");
        uss2 = hardwareMap.ultrasonicSensor.get("uss2");

        ussV = new double[2][10];
        prevA = false;
        count = 0;
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

        if (count < 10) {
            ussV[0][count] = uss1.getUltrasonicLevel();
            ussV[1][count] = uss2.getUltrasonicLevel();

            count++;
        }

        if (gamepad1.a && !prevA)
            count = 0;

        telemetry.addData("Count", count);

        for (int i = 0; i < count; i++) {
            telemetry.addData("I" + i, i);
            telemetry.addData("USSL" + i, ussV[0][i]);
            telemetry.addData("USSR" + i, ussV[1][i]);
        }

    } // PootisBotManual::loop

    @Override public void stop() {
        //cs.enableLed(false);
    }

} // PootisBotManual
