package org.firstinspires.ftc.teamcode.year2016_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Constants;

import for_camera_opmodes.OpModeCamera;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

@Autonomous(name = "Square Test", group = "Autonomous")
@Disabled
public class AutoBotSquare extends OpModeCamera {


// Touch sensor is "ts1".
    DcMotorController mc1;
    DcMotor left, right;
    State state = State.STATE_FIRST;
    UltrasonicSensor uss1, uss2;
    Servo s1, s2;
	VoltageSensor vs;

    DcMotor bll, blr;
    TouchSensor ts1, ts2;

    double powah = -0.2;
    double turnPow = 1.0;

    double ballLaunchSpeed;

    int turnTicks = 450, straightTicks = 900;

    boolean blueOnRight;

    long timeBeforeFive;

    long timeDuringSquare;

    long timeBeforeFive2b;

    long beginningTime, runningTime;
    int color;

    int turnDirection = 1;

    Gamepad prev1;
    boolean gamepad1IsOn;

    /*
    * Code to run when the op mode is first enabled goes here
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
    */
    @Override
    public void init() {
        setCameraDownsampling(8);
        left = hardwareMap.dcMotor.get("lm");
        right = hardwareMap.dcMotor.get("rm");
        mc1 = hardwareMap.dcMotorController.get("mc1");
        uss1 = hardwareMap.ultrasonicSensor.get("uss1");
        uss2 = hardwareMap.ultrasonicSensor.get("uss2");
        ts1 = hardwareMap.touchSensor.get("ts1");
        ts2 = hardwareMap.touchSensor.get("ts2");
        bll = hardwareMap.dcMotor.get("bll");
        blr = hardwareMap.dcMotor.get("blr");
        s1 = hardwareMap.servo.get("s1"); // Launch servo
        s2 = hardwareMap.servo.get("s2"); // Gate servo
		vs = hardwareMap.voltageSensor.get("mc1");

        resetEncoders();

        left.setDirection(Constants.L_MTR_DIR_2016);
        right.setDirection(Constants.R_MTR_DIR_2016);

        state = State.STATE_FIRST;

        timeDuringSquare = 0;

        color = -1;

        super.init(); // inits camera functions, starts preview callback
    }

    @Override
    public void init_loop() {
        if (prev1 == null)
            prev1 = new Gamepad();

        if (gamepad1IsOn) {
            if (gamepad1.a && !prev1.a)
                powah -= 0.01;
            if (gamepad1.b && !prev1.b)
                powah += 0.01;

            if (gamepad1.x && !prev1.x)
                turnPow += 0.05;
            if (gamepad1.y && !prev1.y)
                turnPow -= 0.05;

            if (gamepad1.dpad_right && !prev1.dpad_right)
                turnTicks += 25;
            if (gamepad1.dpad_left && !prev1.dpad_left)
                turnTicks -= 25;

            if (gamepad1.dpad_up && !prev1.dpad_up)
                straightTicks += 25;
            if (gamepad1.dpad_down && !prev1.dpad_down)
                straightTicks -= 25;

            if (gamepad1.left_stick_button && !prev1.left_stick_button)
                turnDirection *= -1;

            try {
                prev1.fromByteArray(gamepad1.toByteArray());
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }

            if (telemetry != null) {
                telemetry.addData("Global Motor Pow.", -powah);
                telemetry.addData("Global Turn Power", turnPow);
                telemetry.addData("Turn Distance", turnTicks);
                telemetry.addData("Straight Section Distance", straightTicks);
                telemetry.addData("Turn Direction", turnDirection > 0 ? "Left" : "Right"); // left power * turnDirection, right power * -turnDirection
                telemetry.addData("Button for Motor Pow.", "A to increase, B to decrease");
                telemetry.addData("Button for Turn Power", "X to increase, Y to decrease");
                telemetry.addData("Button for Turn Distance", "DPad Right to increase, DPad Left to decrease");
                telemetry.addData("Button for Straight Distance", "DPad Up to increase, DPad Down to decrease");
                telemetry.addData("Button for Square Direction", "Press on left stick to switch");
            }
        } else if (gamepad1 != null)
            gamepad1IsOn = true;
    }

    public String colorName(int colorID) {
        switch (colorID) {
            case 0:
                return "RED";
            case 1:
                return "GREEN";
            case 2:
                return "BLUE";
            default:
                return "URINE YELLOW";
        }
    }

    public void resetEncoders() {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getLeftPos() {
        return Math.abs(left.getCurrentPosition());
    }

    public int getRightPos() {
        return Math.abs(right.getCurrentPosition());
    }

    public double getReading(UltrasonicSensor uss) {
        double value;
        do {
            value = uss.getUltrasonicLevel();
        } while (value >= 250 || value <= 7.5);
        return value;
    }

    public enum State {
        STATE_FIRST,
        STATE_STRAIGHT1,
        STATE_TURN1,
        STATE_STRAIGHT2,
        STATE_TURN2,
        STATE_STRAIGHT3,
        STATE_TURN3,
        STATE_STRAIGHT4,
        STATE_TURN4,
        STATE_END,
    }

    /*
    * This method will be called repeatedly in a loop
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
    */
    @Override
    public void loop() {

        switch (state) {
            case STATE_FIRST: // Initialization
                beginningTime = System.currentTimeMillis(); // Used for elapsed time in telemetry.

                left.setPower(powah);
                right.setPower(powah);
                state = State.STATE_STRAIGHT1;
                break;
            case STATE_STRAIGHT1:
                if (getLeftPos() >= straightTicks || getRightPos() >= straightTicks) {
                    left.setPower(powah * turnPow * turnDirection);
                    right.setPower(powah * turnPow * -turnDirection);
                    state = State.STATE_TURN1;
                }
                break;
            case STATE_TURN1:
                if (getLeftPos() >= turnTicks || getRightPos() >= turnTicks) {
                    left.setPower(powah);
                    right.setPower(powah);
                    state = State.STATE_STRAIGHT2;
                }
                break;
            case STATE_STRAIGHT2:
                if (getLeftPos() >= straightTicks || getRightPos() >= straightTicks) {
                    left.setPower(powah * turnPow * turnDirection);
                    right.setPower(powah * turnPow * -turnDirection);
                    state = State.STATE_TURN2;
                }
                break;
            case STATE_TURN2:
                if (getLeftPos() >= turnTicks || getRightPos() >= turnTicks) {
                    left.setPower(powah);
                    right.setPower(powah);
                    state = State.STATE_STRAIGHT3;
                }
                break;
            case STATE_STRAIGHT3:
                if (getLeftPos() >= straightTicks || getRightPos() >= straightTicks) {
                    left.setPower(powah * turnPow * turnDirection);
                    right.setPower(powah * turnPow * -turnDirection);
                    state = State.STATE_TURN3;
                }
                break;
            case STATE_TURN3:
                if (getLeftPos() >= turnTicks || getRightPos() >= turnTicks) {
                    left.setPower(powah);
                    right.setPower(powah);
                    state = State.STATE_STRAIGHT4;
                }
                break;
            case STATE_STRAIGHT4:
                if (getLeftPos() >= straightTicks || getRightPos() >= straightTicks) {
                    left.setPower(powah * turnPow * turnDirection);
                    right.setPower(powah * turnPow * -turnDirection);
                    state = State.STATE_TURN2;
                }
                break;
            case STATE_TURN4:
                if (getLeftPos() >= turnTicks || getRightPos() >= turnTicks) {
                    left.setPower(0);
                    right.setPower(0);
                    state = State.STATE_END;
                }
                break;
            case STATE_END:
                break;
        }

        /*
        telemetry.addData("Left Motor Pos.", getLeftPos());
        telemetry.addData("Right Motor Pos.", getRightPos());

        telemetry.addData("Left Motor Pow.", left.getPower());
        telemetry.addData("Right Motor Pow.", right.getPower());
        */

        telemetry.addData("Left Encoder", getLeftPos());
        telemetry.addData("Right Encoder", getRightPos());
        /*
        telemetry.addData("USS Right Value", getAverage(rightUssValues));
        telemetry.addData("USS Left Value", getAverage(leftUssValues));
        */
		telemetry.addData("Voltage", vs.getVoltage());
		
        telemetry.addData("State", state);

        if (color != -1)
            telemetry.addData("Color", colorName(color));

        /*
        telemetry.addData("Touch 1", ts1.isPressed());
        telemetry.addData("Touch 2", ts2.isPressed());
        telemetry.addData("Speed", observedSpeed);
        */

        telemetry.addData("Ultrasonic Left", uss1.getUltrasonicLevel());
        telemetry.addData("Ultrasonic Right", uss2.getUltrasonicLevel());

        if (!state.equals(State.STATE_END))
            runningTime = System.currentTimeMillis();

        telemetry.addData("Elapsed Time", runningTime - beginningTime);

        telemetry.addData("Image Width", width);
        telemetry.addData("Image Height", height);

    }

    @Override
    public void stop() {
        super.stop(); // stops camera functions
    }
}