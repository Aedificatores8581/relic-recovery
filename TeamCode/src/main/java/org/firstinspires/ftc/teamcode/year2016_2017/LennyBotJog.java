package org.firstinspires.ftc.teamcode.year2016_2017;

//------------------------------------------------------------------------------
//
// PootisBotManual
//

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import java.util.LinkedList;
import java.util.Queue;
import java.util.Timer;

/**
 * I made this from scratch.
 *
 * @author Pootis Man
 * @version 1.0
 */
@TeleOp(name = "LennyBot: Jog 1", group = "( \u0361\u00B0 \u035C\u0296 \u0361\u00B0 )")
@Disabled
public class LennyBotJog extends OpMode
{
    DcMotorController mc1;
    //DcMotorController con;
    //DcMotor motor1, motor2;
    //DcMotor motor3, motor4;
    DcMotor left, right;
    DcMotor first, last;

    //TouchSensor touch1;
    //IrSeekerSensor irseek1;
    //OpticalDistanceSensor uss1;
    UltrasonicSensor uss1;
    UltrasonicSensor uss2;

    boolean prevR, prevL;
    boolean prevA;

    Queue<Double> foreUssValues, leftUssValues;

    boolean movingMode;
    int move = 0;
    double foreUssValueFirst = 0.0;
    boolean moveForwards = false;

    boolean resetEncoder = false;
    Timer timer = new Timer();

    //final double powerMult = -1.0;

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
    public LennyBotJog()
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
        left = hardwareMap.dcMotor.get("lm");
        right = hardwareMap.dcMotor.get("rm");
        mc1 = hardwareMap.dcMotorController.get("mc1");
        uss1 = hardwareMap.ultrasonicSensor.get("uss1");
        uss2 = hardwareMap.ultrasonicSensor.get("uss2");
        //gs1 = hardwareMap.gyroSensor.get("gs1");
        //cs = hardwareMap.colorSensor.get("cs1");
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        //toggleLed = true;
        //cs.enableLed(toggleLed);

        prevR = false;
        prevL = false;
        prevA = false;

        foreUssValues = new LinkedList<>();
        for (int i = 0; i < 16; i++)
            foreUssValues.add(0.0);

        leftUssValues = new LinkedList<>();
        for (int i = 0; i < 16; i++)
            leftUssValues.add(0.0);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        movingMode = false;
        moveForwards = false;
        resetEncoder = false;
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

        if (movingMode) {

            if (moveForwards)
                if (getAverage(foreUssValues) > foreUssValueFirst) {
                    left.setPower(0.04);
                    right.setPower(0.04);
                }
                else {
                    /*timer.schedule(new TimerTask() {
                        @Override
                        public void run() {
                            if (getAverage(foreUssValues) <= foreUssValueFirst) {
                                left.setPower(0.0);
                                right.setPower(0.0);
                                movingMode = false;
                                moveForwards = false;
                                move = 0;
                            }
                        }
                    }, 100);*/

                    left.setPower(0.0);
                    right.setPower(0.0);
                    movingMode = false;
                    moveForwards = false;
                    move = 0;
                }
            else
                if (Math.abs(first.getCurrentPosition()) <= Math.abs(80 * move))
                    first.setPower(-0.1);
                else {
                    first.setPower(0.0);

                    /*if (!resetEncoder) {
                        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        resetEncoder = true;
                    }*/

                    if (Math.abs(last.getCurrentPosition()) <= Math.abs(first.getCurrentPosition()))
                        last.setPower(-0.1);
                    else {
                        last.setPower(0.0);

                        if (!moveForwards) {
                            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                            while ( !(left.getMode() == DcMotor.RunMode.STOP_AND_RESET_ENCODER && right.getMode() == DcMotor.RunMode.STOP_AND_RESET_ENCODER) );

                            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        }

                        moveForwards = true;

                    }
                }

        }
        else {

            if (prevL && !gamepad1.left_bumper)
                move--;

            if (prevR && !gamepad1.right_bumper)
                move++;

            if (prevA && !gamepad1.a) {
                movingMode = true;
                if (move > 0) {
                    first = left;
                    last = right;
                }
                else {
                    first = right;
                    last = left;
                }
                left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                foreUssValueFirst = getAverage(foreUssValues);
            }

        }

        shiftQueue(foreUssValues, uss2.getUltrasonicLevel());
        shiftQueue(leftUssValues, uss1.getUltrasonicLevel());

        prevL = gamepad1.left_bumper;
        prevR = gamepad1.right_bumper;
        prevA = gamepad1.a;

        telemetry.addData("Move Value", move);

        telemetry.addData("Left Uss Value", getAverage(leftUssValues));
        telemetry.addData("Fore Uss Value", getAverage(foreUssValues));

        telemetry.addData("Target Uss Value", foreUssValueFirst);

        telemetry.addData("Moving Mode", movingMode);


    } // PootisBotManual::loop

    @Override public void stop() {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        timer.purge();
    }

} // PootisBotManual
