package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * Created by The Saminator on 06-29-2017.
 */
public abstract class SmolBotTemplate extends OpMode {
    public static class Constants {
        public static final DcMotor.Direction LEFT_DIR = DcMotor.Direction.REVERSE;
        public static final DcMotor.Direction RIGHT_DIR = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction ARM_DIR = DcMotor.Direction.REVERSE;
        public static final DcMotor.Direction HAND_DIR = DcMotor.Direction.FORWARD;
        public static final Servo.Direction GRAB = Servo.Direction.FORWARD;

        public static final double LEFT_SPEED = 0.375; // Always positive and between 0 and 1.
        public static final double RIGHT_SPEED = 0.375; // Always positive and between 0 and 1.
        public static final double ARM_SPEED = 0.50; // Always positive and between 0 and 1.
        public static final double HAND_SPEED = 0.50; // Always positive and between 0 and 1.
        public static final double GRAB_SPEED = 0.25; // Always positive and between 0 and 1.
    }

    DcMotor left, right, arm, hand;
    Servo grab;
    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;
    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("lm");
        right = hardwareMap.dcMotor.get("rm");
        arm = hardwareMap.dcMotor.get("am");
        hand = hardwareMap.dcMotor.get("hm");
        grab = hardwareMap.servo.get("gr");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color sensor");

        left.setDirection(Constants.LEFT_DIR);
        right.setDirection(Constants.RIGHT_DIR);
        arm.setDirection(Constants.ARM_DIR);
        hand.setDirection(Constants.HAND_DIR);
        grab.setDirection(Constants.GRAB);


    }

    @Override
    public void stop() {
        setLeftPow(0.0);
        setRightPow(0.0);

    }

    protected void setLeftPow(double pow) {
        left.setPower(pow * Constants.LEFT_SPEED);
    }
    protected void setRightPow(double pow) {
        right.setPower(pow * Constants.RIGHT_SPEED);
    }

    protected void setArmPow(double pow) {
        arm.setPower(pow * Constants.ARM_SPEED);
    }


    protected void setGrabPow(double position) {
        grab.setPosition(position * Constants.GRAB_SPEED);
    }

    protected void setHandPow(double pow) {
        hand.setPower(pow * Constants.HAND_SPEED);
    }

    protected boolean checkEncoder(int ticks) {
        int distance = Math.abs(ticks);
        int leftDist = Math.abs(left.getCurrentPosition());
        int rightDist = Math.abs(right.getCurrentPosition());

        return (distance <= leftDist) || (distance <= rightDist);
    }
}
