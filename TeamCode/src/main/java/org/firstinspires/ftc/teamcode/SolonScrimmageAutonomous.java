package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Solon Scrimmage Autonomous For Sensor Bot", group = "8581")
@Disabled
public class SolonScrimmageAutonomous extends OpMode {

    private Gamepad prev1;
    private Gamepad prev2;


    private DcMotor rearLeft, rearRight;
    private Servo glyphGrabberRight, glyphGrabberCenter, glyphGrabberLeft, ballSensorArm;

    double position[] = new double[4];

    /************
     * Position for different servos:
     * position[0] ----> glyphGrabberRight
     * position[1] ----> glyphGrabberCenter
     * position[2] ----> glyphGrabberLeft
     * position[3] ----> ballSnsorArm (Never used in TeleOp)*/

    private abstract class SERVO_CONSTANTS {

        protected static final double GRABBER_INCREMENT_VALUE = 0.005;

        // Indexes for the different servos in position[]
        protected static final int GLYPH_GRABBER_INDEX_RIGHT = 0;
        protected static final int GLYPH_GRABBER_INDEX_CENTER = 1;
        protected static final int GLYPH_GRABBER_INDEX_LEFT = 2;
        protected static final int BALL_SENSOR_ARM_INDEX = 3;

        // Initial positions for all the Servos
    }


    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;

    static double redRatio;
    static double blueRatio;


    public enum State {
        STATE_MOVE_DOWN_BALL_SENSOR_ARM,
        STATE_MOVE_UP_BALL_SENSOR_ARM,
        STATE_SCAN_PICTURE,
        STATE_MOVE_PARKING_ZONE,
        STATE_READ_COLOR_SENSOR,
        STATE_DRIVE_TO_SHELF,
        STATE_ORIENT_WITH_SHELF,
        STATE_POSITION_LIFT,
        STATE_INSERT_BLOCK,
        STATE_CHECK_TIME,
        STATE_DRIVE_TO_CENTER,
        STATE_GRAB_BLOCK,
        STATE_DRIVE_BACK,
        STATE_END,
    }

    State state; // current state the robot this is in
    long delay;

    @Override
    public void init() {
        delay = 0L;
        rearLeft = hardwareMap.dcMotor.get("lm");
        rearRight = hardwareMap.dcMotor.get("rm");

        glyphGrabberRight = hardwareMap.servo.get("ggr");
        glyphGrabberCenter = hardwareMap.servo.get("ggc");
        glyphGrabberLeft = hardwareMap.servo.get("ggl");
        ballSensorArm = hardwareMap.servo.get("bsa");

        glyphGrabberRight.setDirection(Servo.Direction.FORWARD);
        glyphGrabberCenter.setDirection(Servo.Direction.REVERSE);
        glyphGrabberLeft.setDirection(Servo.Direction.REVERSE);
        ballSensorArm.setDirection(Servo.Direction.FORWARD);
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "cs");
    }

    @Override
    public void start() {
        state = State.STATE_MOVE_DOWN_BALL_SENSOR_ARM;
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        glyphGrabberLeft.setPosition(.8);
        glyphGrabberRight.setPosition(.8);
    }

    @Override
    public void loop() {
        telemetry.update();

        // telemetry.addData("ballSensorArm", ballSensorArm.getPosition());

        switch (state) {
            case STATE_MOVE_DOWN_BALL_SENSOR_ARM:
                state = State.STATE_READ_COLOR_SENSOR;
                ballSensorArm.setPosition(.052);
                break;

            case STATE_SCAN_PICTURE:
                break;

            case STATE_READ_COLOR_SENSOR:

                boolean doneWithColorSensor = detectColors();
                if (doneWithColorSensor) {
                    state = State.STATE_MOVE_UP_BALL_SENSOR_ARM;
                }

                break;

            case STATE_MOVE_UP_BALL_SENSOR_ARM:
                ballSensorArm.setPosition(.59);
                delay = 0L;
                state = State.STATE_MOVE_PARKING_ZONE;
                break;

            case STATE_MOVE_PARKING_ZONE:



                break;

            default:
                break;

        }

        telemetry.addData("STATE", state);


    }


    @Override
    public void stop() {
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private boolean detectColors() {
        colors = colorSensor.getNormalizedColors();
        redRatio = colors.red / (colors.red + colors.blue + colors.green);
        blueRatio = colors.blue / (colors.red + colors.blue + colors.green);


        boolean redAliance = true;
        boolean hasFlipped = false;

        if (redAliance == true) {
            if (redRatio >= 0.55 && redRatio > blueRatio) {
                if (delay == 0L) {
                    delay = System.currentTimeMillis();
                }
                if (System.currentTimeMillis() - delay < 400) {
                    setRightPow(-1.0);
                    setLeftPow(1.0);
                } else {
                        hasFlipped = true;
                        setRightPow(0.0);
                        setLeftPow(0.0);
                }
            } else if (blueRatio >= 0.4 && redRatio < blueRatio) {
                if (delay == 0L) {
                    delay = System.currentTimeMillis();
                }
                if (System.currentTimeMillis() - delay < 400) {
                    setRightPow(1.0);
                    setLeftPow(-1.0);
                } else
                    hasFlipped = true;
                setRightPow(0.0);
                setLeftPow(0.0);
                }
            }

        if (redAliance == false) {
            if (redRatio >= 0.55 && redRatio > blueRatio) {
                if (delay == 0L) {
                    delay = System.currentTimeMillis();
                }
                if (System.currentTimeMillis() - delay < 400) {

                    setRightPow(1.0);
                    setLeftPow(-1.0);
                } else
                    hasFlipped = true;
                    setRightPow(0.0);
                    setLeftPow(0.0);
                }
            } else if (blueRatio >= 0.4 && redRatio < blueRatio) {
            if (delay == 0L) {
                delay = System.currentTimeMillis();
            }
            if (System.currentTimeMillis() - delay < 400) {

                setRightPow(-1.0);
                setLeftPow(1.0);
            } else {

                hasFlipped = true;
                setRightPow(0.0);
                setLeftPow(0.0);

            }
        }


        telemetry.addData("Left", rearLeft.getCurrentPosition());
        telemetry.addData("Right", rearRight.getCurrentPosition());
        telemetry.addLine()
                .addData("a", colors.alpha)
                .addData("red Ratio", (colors.red / (colors.blue + colors.red + colors.green)))
                .addData("green Ratio", (colors.green / (colors.blue + colors.red + colors.green)))
                .addData("blue Ratio", (colors.blue / (colors.blue + colors.red + colors.green)))
                .addData("blue", colors.blue)
                .addData("red", colors.red);
        telemetry.update();
        return hasFlipped;

    }


    protected void setLeftPow(double pow) {
        rearLeft.setPower(pow * SmolBotTemplate.Constants.LEFT_SPEED);
    }

    protected void setRightPow(double pow) {
        rearRight.setPower(pow * SmolBotTemplate.Constants.RIGHT_SPEED);
    }

    protected boolean checkEncoder(int ticks) {
        int distance = Math.abs(ticks);
        int rearLeftDist = Math.abs(rearLeft.getCurrentPosition());
        int rearRightDist = Math.abs(rearRight.getCurrentPosition());

        return (distance <= rearLeftDist) || (distance <= rearRightDist);
    }

}
