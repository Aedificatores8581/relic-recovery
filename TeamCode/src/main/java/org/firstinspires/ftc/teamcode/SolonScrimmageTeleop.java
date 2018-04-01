package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * Created by Mister-Minister-Master on 11/12/2017.
 */
@Disabled
@TeleOp(name = "Solon Scrimmage TeleOp For Sensor Bot", group = "8581")
public class SolonScrimmageTeleop extends OpMode {

    private DcMotor rearLeft, rearRight;
    private Servo glyphGrabberRight, glyphGrabberCenter, glyphGrabberLeft, ballSensorArm;

    double position[] = new double[4];

    /************
     * Position for different servos:
     * position[0] ----> glyphGrabberRight
     * position[1] ----> glyphGrabberCenter
     * position[2] ----> glyphGrabberLeft
     * position[3] ----> ballSnsorArm (Never used in TeleOp)
     */

    private abstract class SERVO_CONSTANTS {

        protected static final double GRABBER_INCREMENT_VALUE = 0.005;

        // Indexes for the different servos in position[]
        protected static final int GLYPH_GRABBER_INDEX_RIGHT = 0;
        protected static final int GLYPH_GRABBER_INDEX_CENTER = 1;
        protected static final int GLYPH_GRABBER_INDEX_LEFT = 2;
        protected static final int BALL_SENSOR_ARM_INDEX = 3;

        // Initial positions for all the Servos
    }


    ColorSensor colorSensor;

    public void init() {

        rearLeft = hardwareMap.dcMotor.get("lm");
        rearRight = hardwareMap.dcMotor.get("rm");


        glyphGrabberRight = hardwareMap.servo.get("ggr");
        glyphGrabberCenter = hardwareMap.servo.get("ggc");
        glyphGrabberLeft = hardwareMap.servo.get("ggl");
        ballSensorArm = hardwareMap.servo.get("bsa");

        glyphGrabberRight.setDirection(Servo.Direction.FORWARD);
        glyphGrabberCenter.setDirection(Servo.Direction.REVERSE);
        glyphGrabberLeft.setDirection(Servo.Direction.REVERSE);
        ballSensorArm.setDirection(Servo.Direction.REVERSE);
        colorSensor = hardwareMap.colorSensor.get("cs");


    }

    public void start() {
        glyphGrabberRight.setPosition(0.0);
        glyphGrabberCenter.setPosition(0.0);
        glyphGrabberLeft.setPosition(0.0);

    }

    public void loop() {
        telemetry.clear();
        telemetry.update();

        position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_RIGHT] = glyphGrabberRight.getPosition();
        position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_CENTER] = glyphGrabberCenter.getPosition();
        position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_LEFT] = glyphGrabberLeft.getPosition();
        position[SERVO_CONSTANTS.BALL_SENSOR_ARM_INDEX] = ballSensorArm.getPosition();

        if (Math.abs(gamepad1.left_stick_y) > .5) {
            setRightPow(gamepad1.left_stick_y * 1.2);
        } else {
            setRightPow(0.0);
        }
        if (Math.abs(gamepad1.right_stick_y) > .5) {
            setLeftPow(-gamepad1.right_stick_y * 1.2);
        } else {
            setLeftPow(0.0);
        }

        if (gamepad1.left_bumper) {
            telemetry.addLine("L1 Down");

            position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_CENTER] -= SERVO_CONSTANTS.GRABBER_INCREMENT_VALUE;

            glyphGrabberCenter.setPosition(position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_CENTER]);

        } else if (gamepad1.left_trigger > .5) {
            telemetry.addLine("L2 Down");

            position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_CENTER] += SERVO_CONSTANTS.GRABBER_INCREMENT_VALUE;

            glyphGrabberCenter.setPosition(position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_CENTER]);

        }

        if (gamepad1.right_bumper) {
            telemetry.addLine("Pressing right_bumper");
            position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_LEFT] += SERVO_CONSTANTS.GRABBER_INCREMENT_VALUE;
            position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_RIGHT] += SERVO_CONSTANTS.GRABBER_INCREMENT_VALUE;
            telemetry.addData("position[right]", position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_RIGHT]);

            glyphGrabberRight.setPosition(position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_RIGHT]);
            telemetry.addData("position[left]", position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_LEFT]);

            glyphGrabberLeft.setPosition(position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_LEFT]);
        } else if (gamepad1.right_trigger > .5) {
            telemetry.addLine("Pressing right_trigger");
            position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_LEFT] -= SERVO_CONSTANTS.GRABBER_INCREMENT_VALUE;
            telemetry.addData("position[left]", position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_LEFT]);
            position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_RIGHT] -= SERVO_CONSTANTS.GRABBER_INCREMENT_VALUE;
            telemetry.addData("position[right]", position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_RIGHT]);

            glyphGrabberRight.setPosition(position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_RIGHT]);
            glyphGrabberLeft.setPosition(position[SERVO_CONSTANTS.GLYPH_GRABBER_INDEX_LEFT]);
        }


        if (gamepad1.x) {
            position[SERVO_CONSTANTS.BALL_SENSOR_ARM_INDEX] += SERVO_CONSTANTS.GRABBER_INCREMENT_VALUE;

            ballSensorArm.setPosition(position[SERVO_CONSTANTS.BALL_SENSOR_ARM_INDEX]);
        } else if (gamepad1.a) {
            position[SERVO_CONSTANTS.BALL_SENSOR_ARM_INDEX] -= SERVO_CONSTANTS.GRABBER_INCREMENT_VALUE;

            ballSensorArm.setPosition(position[SERVO_CONSTANTS.BALL_SENSOR_ARM_INDEX]);
        }


        telemetry.addData("Right glyph: ", glyphGrabberRight.getPosition());
        telemetry.addData("Center glyph: ", glyphGrabberCenter.getPosition());
        telemetry.addData("Left glyph: ", glyphGrabberLeft.getPosition());
        telemetry.addData("ball sensor: ", ballSensorArm.getPosition());


        //ballSensorArm.setPosition(position[SERVO_CONSTANTS.BALL_SENSOR_ARM_INDEX]);

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
