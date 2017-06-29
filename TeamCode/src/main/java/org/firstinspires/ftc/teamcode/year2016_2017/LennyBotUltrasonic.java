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
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import java.util.LinkedList;
import java.util.Queue;

/**
 * I made this from scratch.
 *
 * @author Pootis Man
 * @version 1.0
 */
@Autonomous(name = "LennyBot: Ultrasonic Sensor", group = "( \u0361\u00B0 \u035C\u0296 \u0361\u00B0 )")
@Disabled
public class LennyBotUltrasonic extends OpMode
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
    int targetPos, direction;
    int position;
    UltrasonicSensor uss1, uss2;
    Queue<Double> leftUssValues;
    Queue<Double> rightUssValues;
    //-------------------------------------------------------------------------
    //
    // PootisBotManual
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------
    public LennyBotUltrasonic()
    {


    } // PootisBotManual::PootisBotManual

    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("lm");
        right = hardwareMap.dcMotor.get("rm");
        mc1 = hardwareMap.dcMotorController.get("mc1");
        uss1 = hardwareMap.ultrasonicSensor.get("uss1");
        uss2 = hardwareMap.ultrasonicSensor.get("uss2");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        state = 0;
        leftUssValues = new LinkedList<>();
        rightUssValues = new LinkedList<>();
        for (int i = 0; i < 64; i++)
            leftUssValues.add(0.0);
        for (int i = 0; i < 64; i++)
            rightUssValues.add(0.0);

    }

    public void setMotorPower(double pow) {
        left.setPower(pow);
        right.setPower(pow);
    }


    public double getAverage(Queue<Double> dubs) {
        double sum = 0;
        int num = 0;
        for (double d : dubs) {
            sum += d;
            if (d != 0.0)
                num++;
        }
        return sum / (double)(num);
    }

    public void shiftQueue(Queue<Double> dubs, double dub) {
        if (dub != 0.0) {
            dubs.poll();
            dubs.add(dub);
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
        if (getAverage(rightUssValues) >= 20) {
            left.setPower(0.1);
            right.setPower(left.getPower() * getAverage(leftUssValues) / 59);
        } else setMotorPower(0.0);

        shiftQueue(leftUssValues, uss1.getUltrasonicLevel());
        shiftQueue(rightUssValues, uss2.getUltrasonicLevel());

        telemetry.addData("USS Left Value", getAverage(leftUssValues));
        telemetry.addData("USS Right Value", getAverage(rightUssValues));

        telemetry.addData("USS Left Value (Raw)", uss1.getUltrasonicLevel());
        telemetry.addData("USS Right Value (Raw)", uss2.getUltrasonicLevel());



    } // PootisBotManual::loop

} // PootisBotManual
