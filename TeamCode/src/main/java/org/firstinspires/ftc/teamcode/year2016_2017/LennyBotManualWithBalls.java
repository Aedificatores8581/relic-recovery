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
import com.qualcomm.robotcore.hardware.Servo;
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
@TeleOp(name = "LennyBot: Tele Balls", group = "( \u0361\u00B0 \u035C\u0296 \u0361\u00B0 )")
@Disabled
public class LennyBotManualWithBalls extends OpMode
{
    DcMotorController mc1;
    //DcMotorController con;
    //DcMotor motor1, motor2;
    //DcMotor motor3, motor4;
    DcMotor left, right;
    DcMotor ballLauncherLeft, ballLauncherRight;
    //TouchSensor touch1;
    //IrSeekerSensor irseek1;
    //OpticalDistanceSensor uss1;
    Servo s1, s2;
    double s1position, s2position;
    UltrasonicSensor uss1;
    UltrasonicSensor uss2;
    //GyroSensor gs1;
    //ColorSensor cs;
    double rightSpeed = 1.0, leftSpeed = 1.0;
    double ballMult = 1.0;

    boolean prevR, prevL;
    double prevTR, prevTL, prevTL2, prevTR2;
    boolean prevB;

    Queue<Double> leftUssValues;
    Queue<Double> rightUssValues;

    double obsLeftSpeed, obsRightSpeed;

    SpeedTimerTask.SpeedReceiver leftSpeedRecv = new SpeedTimerTask.SpeedReceiver() {
        @Override
        public void setSpeed(double speed) {
            obsLeftSpeed = speed;
        }
    }, rightSpeedRecv = new SpeedTimerTask.SpeedReceiver() {
        @Override
        public void setSpeed(double speed) {
            obsRightSpeed = speed;
        }
    };

    SpeedTimerTask.PositionProvider leftPositionProv = new SpeedTimerTask.PositionProvider() {
        @Override
        public double getPosition() {
            return left.getCurrentPosition();
        }
    }, rightPositionProv = new SpeedTimerTask.PositionProvider() {
        @Override
        public double getPosition() {
            return right.getCurrentPosition();
        }
    };

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
    public LennyBotManualWithBalls()
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
        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");
        uss1 = hardwareMap.ultrasonicSensor.get("uss1");
        uss2 = hardwareMap.ultrasonicSensor.get("uss2");
        //gs1 = hardwareMap.gyroSensor.get("gs1");
        //cs = hardwareMap.colorSensor.get("cs1");
        ballLauncherLeft = hardwareMap.dcMotor.get("bll");
        ballLauncherRight = hardwareMap.dcMotor.get("blr");
        s1position = 0.0;
        s2position = 0.0;
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        ballLauncherLeft.setDirection(DcMotor.Direction.FORWARD);
        ballLauncherRight.setDirection(DcMotor.Direction.FORWARD);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //toggleLed = true;
        //cs.enableLed(toggleLed);

        rightSpeed = 0.4;
        leftSpeed = 0.4;

        prevR = false;
        prevL = false;
        prevB = false;

        prevTL = 0.0;
        prevTR = 0.0;

        leftUssValues = new LinkedList<>();
        rightUssValues = new LinkedList<>();
        for (int i = 0; i < 64; i++)
            leftUssValues.add(0.0);
        for (int i = 0; i < 64; i++)
            rightUssValues.add(0.0);

        //gs1.calibrate();
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

        double ballLauncherPower = Math.max(0.0, gamepad2.right_stick_y);

        left.setPower(-gamepad1.left_stick_y * leftSpeed);
        right.setPower(-gamepad1.right_stick_y * rightSpeed);
        s1.setPosition(s1position);
        s2.setPosition(s2position);

        timer.schedule(new SpeedTimerTask(left.getCurrentPosition(), 250, leftPositionProv, leftSpeedRecv, timer), 250);
        timer.schedule(new SpeedTimerTask(right.getCurrentPosition(), 250, rightPositionProv, rightSpeedRecv, timer), 250);

        if (gamepad2.left_bumper) {
            if (s1position < 1.0)
                s1position += 0.05;
        }
        else {
            if (s1position > 0.0)
                s1position -= 0.05;
        }

        if (!gamepad2.right_bumper) {
            if (s2position < 1.0)
                s2position += 0.05;
        }
        else {
            if (s2position > 0.0)
                s2position -= 0.05;
        }

        if (gamepad1.right_bumper && !prevR)
            if (rightSpeed <= 0.95)
                rightSpeed += 0.05;
        if (gamepad1.left_bumper && !prevL)
            if (rightSpeed >= 0.05)
                rightSpeed -= 0.05;

        if ((gamepad1.left_trigger >= 0.5) && (prevTL < 0.5))
            if (leftSpeed >= 0.05)
                leftSpeed -= 0.05;
        if ((gamepad1.right_trigger >= 0.5) && (prevTR < 0.5))
            if (leftSpeed <= 0.95)
                leftSpeed += 0.05;

        if ((gamepad2.left_trigger >= 0.5) && (prevTL2 < 0.5))
            if (ballMult >= 0.05)
                ballMult -= 0.05;
        if ((gamepad2.right_trigger >= 0.5) && (prevTR2 < 0.5))
            if (ballMult <= 0.95)
                ballMult += 0.05;

        ballLauncherLeft.setPower(ballLauncherPower * ballMult);
        ballLauncherRight.setPower(ballLauncherPower * ballMult);

        //if (gamepad1.b && !prevB2) {
        //toggleLed = !toggleLed;
        //cs.enableLed(toggleLed);
        //}

        telemetry.addData("Left Motor Power", left.getPower());
        telemetry.addData("Right Motor Power", right.getPower());

        telemetry.addData("Left Motor Pos.", left.getCurrentPosition());
        telemetry.addData("Right Motor Pos.", right.getCurrentPosition());
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Stick Y", gamepad1.right_stick_y);

        telemetry.addData("S1 Position", s1position);
        //telemetry.addData("Light Sensor", cs.getLightDetected());
        //telemetry.addData("Color Sensor - Red", cs.red());
        //telemetry.addData("Color Sensor - Green", cs.green());
        //telemetry.addData("Color Sensor - Blue", cs.blue());

        //telemetry.addData("Color Sensor LED", toggleLed);

        telemetry.addData("Right Speed", rightSpeed);
        telemetry.addData("Left Speed", leftSpeed);

        //telemetry.addData("Rotation Heading", gs1.getHeading());
        //telemetry.addData("Rotation Fraction", gs1.getRotationFraction());

        shiftQueue(leftUssValues, uss1.getUltrasonicLevel());
        shiftQueue(rightUssValues, uss2.getUltrasonicLevel());

        telemetry.addData("USS Left Level", (int) getAverage(leftUssValues));
        telemetry.addData("USS Right Level", (int) getAverage(rightUssValues));

        prevR = gamepad1.right_bumper;
        prevL = gamepad1.left_bumper;
        prevTR = gamepad1.right_trigger;
        prevTL = gamepad1.left_trigger;

        prevB = gamepad1.b;

        prevTL2 = gamepad2.left_trigger;
        prevTR2 = gamepad2.right_trigger;

        telemetry.addData("Obs. Left Speed", obsLeftSpeed);
        telemetry.addData("Obs. Right Speed", obsRightSpeed);
        telemetry.addData("Ball Shooter Speed", ballMult);


    } // PootisBotManual::loop

    @Override public void stop() {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //cs.enableLed(false);
    }

} // PootisBotManual
