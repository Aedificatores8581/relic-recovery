package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by The Saminator on 11-12-2016.
 */
public class Constants {
    public static final DcMotor.Direction LEFT_DIR = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction RIGHT_DIR = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction ARM_DIR = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction HAND_DIR = DcMotor.Direction.FORWARD;
    public static final Servo.Direction GRAB = Servo.Direction.FORWARD;

    public static final double LEFT_SPEED = 0.375; // Always positive and between 0 and 1.
    public static final double RIGHT_SPEED = 0.375; // Always positive and between 0 and 1.
    public static final double ARM_SPEED = 0.50; // Always positive and between 0 and 1.
    public static final double HAND_SPEED = 0.50; // Always positive and between 0 and 1.
    public static final double GRAB_RANGE = 0.25; // Always positive and between 0 and 1.
    public static final double COLOR_ARM_SPEED = 1; // Always positive and between 0 and 1.

    public static final double RADS_TO_DEGS = 180.0 / Math.PI;
    public static final double DEGS_TO_RADS = 1 / RADS_TO_DEGS;

    public static final double GLYPH_DISPENSE_LEVEL = 0.4;
    public static final double RED_THRESHOLD = 0.4;
    public static final double BLUE_THRESHOLD = 0.3;


    public static final double JEWEL_ARM_DOWN_POSITION = 0.74;
}
