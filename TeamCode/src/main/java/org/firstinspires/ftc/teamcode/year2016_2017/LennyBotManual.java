package org.firstinspires.ftc.teamcode.year2016_2017;

//------------------------------------------------------------------------------
//
// PootisBotManual
//

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.LinkedList;
import java.util.Queue;
import java.util.Timer;
import java.util.TimerTask;

/**
 * I made this from scratch.
 *
 * @author Pootis Man
 * @version 1.0
 */
@TeleOp(name = "LennyBot: Tele Test 1", group = "( \u0361\u00B0 \u035C\u0296 \u0361\u00B0 )")
public class LennyBotManual extends OpMode
{
    TouchSensor ts1, ts2;
    DcMotorController mc1, mc2, mc3;
    //DcMotorController con;
    //DcMotor motor1, motor2;
    //DcMotor motor3, motor4;
    DcMotor left, right;
    //TouchSensor touch1;
    //IrSeekerSensor irseek1;
    //OpticalDistanceSensor uss1;
    Servo s1, s2;
    ServoController sc1;
    DcMotor bll, blr;
    DcMotor sm;
    UltrasonicSensor uss1;
    UltrasonicSensor uss2;
    //GyroSensor gs1;
    //ColorSensor cs;
    double motorSpeed = 1.0;
    Gamepad prev1, prev2;

    boolean sweepBalls;

    boolean dlORr;

    double s1StartPosition;

    Queue<Double> leftUssValues;
    Queue<Double> rightUssValues;

    double ballLaunchSpeed;

    VoltageSensor vs;

    boolean canChangeServo;

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
    public LennyBotManual()
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
        mc2 = hardwareMap.dcMotorController.get("mc2");
        mc3 = hardwareMap.dcMotorController.get("mc3");
        vs = hardwareMap.voltageSensor.get("mc1");
        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");
        sc1 = hardwareMap.servoController.get("sc1");
        sm = hardwareMap.dcMotor.get("sm");
        bll = hardwareMap.dcMotor.get("bll");
        blr = hardwareMap.dcMotor.get("blr");
        uss1 = hardwareMap.ultrasonicSensor.get("uss1");
        uss2 = hardwareMap.ultrasonicSensor.get("uss2");
        //gs1 = hardwareMap.gyroSensor.get("gs1");
        //cs = hardwareMap.colorSensor.get("cs1");
        canChangeServo = true;
        s1.setPosition(Constants.SERVO_MIN_2016);
        s2.setPosition(Constants.SRV2_MIN_2016);
        ts1 = hardwareMap.touchSensor.get("ts1");
        ts2 = hardwareMap.touchSensor.get("ts2");
        left.setDirection(Constants.L_MTR_DIR_2016);
        right.setDirection(Constants.R_MTR_DIR_2016);
        sweepBalls = false;
        bll.setPower(0);
        blr.setPower(0);
        sm.setDirection(DcMotor.Direction.FORWARD);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //toggleLed = true;
        //cs.enableLed(toggleLed);

        motorSpeed = 0.4;

        prev1 = new Gamepad();
        prev2 = new Gamepad();

        s1StartPosition = s1.getPosition();

        leftUssValues = new LinkedList<>();
        rightUssValues = new LinkedList<>();
        for (int i = 0; i < 64; i++)
            leftUssValues.add(0.0);
        for (int i = 0; i < 64; i++)
            rightUssValues.add(0.0);

        s1.setPosition(Constants.SERVO_MIN_2016);
        s2.setPosition(1.0);
        dlORr = true;
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

    @Override
    public void start() {
        if (vs.getVoltage() > 13.5)
            ballLaunchSpeed = 0.28 * Constants.BALL_LAUNCHER_DIR_2016;
        else if (vs.getVoltage() > 13.2)
            ballLaunchSpeed = 0.29 * Constants.BALL_LAUNCHER_DIR_2016;
        else
            ballLaunchSpeed = 0.30 * Constants.BALL_LAUNCHER_DIR_2016;
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
                left.setPower(gamepad1.left_stick_y * motorSpeed);
                right.setPower(gamepad1.right_stick_y * motorSpeed);
                if (gamepad1.dpad_left && dlORr) {
                    ballLaunchSpeed -= .1;
                    dlORr = false;
                    timer.schedule(new TimerTask() {
                        @Override
                        public void run() {
                            dlORr = true;
                        }
                    }, 500);
                }
                if (gamepad1.dpad_right && dlORr) {
                    ballLaunchSpeed += 0.1;
                    dlORr = false;
                    timer.schedule(new TimerTask() {
                        @Override
                        public void run() {
                            dlORr = true;
                        }
                    }, 500);
                }

                    if (canChangeServo) {
                        s1.setPosition(Math.max(Constants.SERVO_MIN_2016, Math.min(s1.getPosition() - (gamepad2.left_stick_y * 0.01), Constants.SERVO_MAX_2016)));
                    }

                if (gamepad1.dpad_up && !prev1.dpad_up) {
                    //if (s2.getPosition() <= 0.95)
                    s2.setPosition(s2.getPosition() + 0.05);
        }
        if (gamepad1.dpad_down && !prev1.dpad_down) {
            s2.setPosition(s2.getPosition() - 0.05);
        }

        /*timer.schedule(new SpeedTimerTask(left.getCurrentPosition(), 250, leftPositionProv, leftSpeedRecv, timer), 250);
        timer.schedule(new SpeedTimerTask(right.getCurrentPosition(), 250, rightPositionProv, rightSpeedRecv, timer), 250);*/

        if (gamepad1.right_bumper) {
            sm.setDirection(DcMotor.Direction.FORWARD);
            sm.setPower(1.0);
        }
        else if (gamepad1.left_bumper) {
            sm.setDirection(DcMotor.Direction.REVERSE);
            sm.setPower(1.0);
        }
        else if (gamepad1.right_trigger > 0.5f) {
            sm.setDirection(DcMotor.Direction.FORWARD);
            sm.setPower(1.0);
        }
        else if (gamepad1.left_trigger > 0.5f) {
            sm.setDirection(DcMotor.Direction.REVERSE);
            sm.setPower(1.0);
        }
        else
            sm.setPower(0.0);
        /*if ((gamepad2.b && !prev2.b) || (gamepad1.b && !prev1.b))
            sweepBalls = !sweepBalls;*/

        if (gamepad1.x && !prev1.x) {
            bll.setPower(-ballLaunchSpeed * Constants.BALL_LAUNCHER_DIR_2016);
            blr.setPower(-ballLaunchSpeed * Constants.BALL_LAUNCHER_DIR_2016);
            canChangeServo = false;
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    s1.setPosition(Constants.SERVO_MAX_2016);
                    timer.schedule(new TimerTask() {
                        @Override
                        public void run() {
                            s1.setPosition(Constants.SERVO_MIN_2016);
                            bll.setPower(0);
                            blr.setPower(0);
                            canChangeServo = true;
                        }
                    }, 1000);
                }
            }, 1000);
        }

        //if (gamepad2.y && !prev2.y)
        //    s2.setPosition((s2.getPosition() >= Constants.SRV2_MAX) ? Constants.SRV2_MIN : Constants.SRV2_MAX);

        //if (gamepad1.b && !prevB2) {
        //toggleLed = !toggleLed;
        //cs.enableLed(toggleLed);
        //}

        telemetry.addData("Left Motor Power", left.getPower());
        telemetry.addData("Right Motor Power", right.getPower());

        telemetry.addData("Ball Launcher Motor Power", bll.getPower());

        telemetry.addData("Left Motor Pos.", left.getCurrentPosition());
        telemetry.addData("Right Motor Pos.", right.getCurrentPosition());
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Stick Y", gamepad1.right_stick_y);

        telemetry.addData("Servo Position", s1.getPosition());
        //telemetry.addData("Light Sensor", cs.getLightDetected());
        //telemetry.addData("Color Sensor - Red", cs.red());
        //telemetry.addData("Color Sensor - Green", cs.green());
        //telemetry.addData("Color Sensor - Blue", cs.blue());

        //telemetry.addData("Color Sensor LED", toggleLed);

        telemetry.addData("S2 Position", s2.getPosition());

        telemetry.addData("Right Speed", motorSpeed);

        //telemetry.addData("Rotation Heading", gs1.getHeading());
        //telemetry.addData("Rotation Fraction", gs1.getRotationFraction());

        shiftQueue(leftUssValues, uss1.getUltrasonicLevel());
        shiftQueue(rightUssValues, uss2.getUltrasonicLevel());

        telemetry.addData("USS Left Level", (int) getAverage(leftUssValues));
        telemetry.addData("USS Right Level", (int) getAverage(rightUssValues));

        try {
            prev1.fromByteArray(gamepad1.toByteArray());
            prev2.fromByteArray(gamepad2.toByteArray());
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }

        telemetry.addData("Obs. Left Speed", obsLeftSpeed);
        telemetry.addData("Obs. Right Speed", obsRightSpeed);

        telemetry.addData("Touch Sensor One", ts1.isPressed());
        telemetry.addData("Touch Sensor Two", ts2.isPressed());
        telemetry.addData("Launcher Motor Speed", ballLaunchSpeed);

    } // PootisBotManual::loop

    @Override public void stop() {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //cs.enableLed(false);
    }

} // PootisBotManual
