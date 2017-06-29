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
@Autonomous(name = "LennyBot: Wall Follower", group = "( \u0361\u00B0 \u035C\u0296 \u0361\u00B0 )")
@Disabled
public class LennyBotWallFollower extends OpMode
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
    Queue<Double> leftUssValues; // Side
    Queue<Double> rightUssValues; // Front

    final int THRESHOLD = 59;


    //-------------------------------------------------------------------------
    //
    // PootisBotManual
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------
    public LennyBotWallFollower()
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

    public boolean wallLeft() {
        return getAverage(leftUssValues) <= THRESHOLD;
    }

    public boolean wallMid() {
        return getAverage(rightUssValues) <= THRESHOLD;
    }

    public boolean tooCloseLeft() {
        return getAverage(leftUssValues) <= 58;
    }

    public boolean tooCloseRight() {
        return getAverage(rightUssValues) <= 58;
    }

    public void moveMotors(double leftPow, double rightPow) {
        left.setPower(leftPow);
        right.setPower(rightPow);
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
        shiftQueue(leftUssValues, uss1.getUltrasonicLevel());
        shiftQueue(rightUssValues, uss2.getUltrasonicLevel());

        telemetry.addData("USS Left Value", getAverage(leftUssValues));
        telemetry.addData("USS Right Value", getAverage(rightUssValues));

        telemetry.addData("USS Left Value (Raw)", uss1.getUltrasonicLevel());
        telemetry.addData("USS Right Value (Raw)", uss2.getUltrasonicLevel());

        if (wallLeft() && wallMid())
            moveMotors(0.1, 0.0);

        if (wallLeft() && !wallMid())
            moveMotors(0.1, 0.1);

        if (!wallLeft() && wallMid())
            moveMotors(0.1, 0.0);

        if (!wallLeft() && !wallMid())
            moveMotors(0.05, 0.1);

        if (tooCloseLeft() && !wallMid())
            moveMotors(0.1, 0.05);

    } // PootisBotManual::loop

} // PootisBotManual
