package org.firstinspires.ftc.teamcode;

import java.lang.Object.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.math.*;
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import java.util.Locale;

import java.util.LinkedList;
import java.util.Queue;
import java.util.Timer;
import java.util.TimerTask;

/**
 * I made this from scratch.
 *
 * @author Pootis Man
 */
@TeleOp(name = "Drive_Forward", group = "bepis")

@Disabled
public class DriveForward extends DriveForwardTemp
{

    //--------------------------------------------------------------------------
    //
    //
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------



    @Override
    public void start() {
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
    double setLeft = 0.0;
    double setRight = 0.0;
    double drive = 0.0;
    double diameter = 90.0;
    double fullSpeed = 0.0;
    double tick = 0.0;
    long time = 0;
    double revolution = 0.0;
    double leftTick = 0.0;
    double rightTick = 0.0;
    @Override public void loop () {
        double leftY = gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightY = gamepad1.right_stick_y;
        double rightX = gamepad1.right_stick_x;
        if (leftY > 0 && leftX > 0 && leftY > leftX)
            setLeft = 1.0;
        else if (leftY > 0 && leftX > 0 && leftX > leftY)
            setLeft = 2.0;
        else if (leftY < 0 && leftX > 0 && leftX > (leftY * -1))
            setLeft = 3.0;
        else if (leftY < 0 && leftX >= 0 && leftX < (leftY * -1))
            setLeft = 4.0;
        else if (leftY < 0 && leftX < 0 && (leftX * -1) > (leftY * -1))
            setLeft = 5.0;
        else if (leftY < 0 && leftX < 0 && (leftX * -1) < (leftY * -1))
            setLeft = 6.0;
        else if (leftY > 0 && leftX < 0 && (leftX * -1) > leftY)
            setLeft = 7.0;
        else if (leftY > 0 && leftX < 0 && (leftX * -1) < leftY)
            setLeft = 8.0;
        else if (leftY == 0 && leftX == 0)
            setLeft = 0;


        if (rightY > 0 && rightX > 0 && rightY > rightX)
            setRight = 1.0 / 8;
        else if ((rightY > 0 && rightX > 0 && rightX > rightY) || (rightX == 0))
            setRight = 2.0 / 8;
        else if (rightY < 0 && rightX > 0 && rightX > (rightY * -1))
            setRight = 3.0 / 8;
        else if (rightY < 0 && rightX > 0 && rightX < (rightY * -1))
            setRight = 4.0 / 8;
        else if (rightY < 0 && rightX < 0 && (rightX * -1) > (rightY * -1))
            setRight = 5.0 / 8;
        else if (rightY < 0 && rightX < 0 && (rightX * -1) < (rightY * -1))
            setRight = 6.0 / 8;
        else if (rightY > 0 && rightX < 0 && (rightX * -1) > rightY)
            setRight = 7.0 / 8;
        else if (rightY > 0 && rightX < 0 && (rightX * -1) < rightY)
            setRight = 8.0 / 8;
        else if (rightY == 0 && rightX == 0)
            setRight = 0;
        drive = setLeft + setRight;
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (gamepad1.a)
            setRightPow(0.25);
            setLeftPow(0.25);
        leftTick = left.getCurrentPosition();
        rightTick = right.getCurrentPosition();
        telemetry.addData("left encoder = ", leftTick);
        telemetry.addData("right encoder = ", rightTick);
        if (gamepad1.left_bumper && gamepad1.right_bumper)

            //using encoder ticks
            tick = revolution * drive;
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //drive forward
        if (checkEncoder((int)tick));
        //stop driving
    }
   /* public void sleep(double millis)
            throws InterruptedException {


            //using
     /*       time = (new Double(fullSpeed * drive / (diameter * Math.PI))).longValue();


        //drive forward

        Thread.sleep(time);
        //stop driving forward
        while (millis < fullSpeed * drive / (diameter * Math.PI)) {
            if (Thread.interrupted()) {
                throw new InterruptedException();
    */


            }







       // }

// PootisBotManuall