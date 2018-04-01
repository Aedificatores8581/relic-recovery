package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------------------
//
// PootisBotManual
//

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Autonomous(name = "RevBot: Autonomous", group = "bepis")

@Disabled
public class SmolBotAutonomous extends SmolBotTemplate
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
    public void init() {
        super.init();
    }


    public SmolBotAutonomous() {
        //
        // Initialize base classes.
        //
        // All via self-construction.
        //
        // Initialize class members.
        //
        // All via self-construction.

    }

    double armpower = 0.8;
    double handpower = 0.8;
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

    @Override public void loop () {
        colors = colorSensor.getNormalizedColors();
        double left = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;
        double green = colors.green;
        double blue = colors.blue;
        double red = colors.red;
        setLeftPow(left);
        setRightPow(right);
        

/*        if (gamepad1.left_stick_button)
            setGrabPow(1);
        else if (gamepad1.right_stick_button)
            setGrabPow(-1);
        else
            setGrabPow(0);
*/
        if (gamepad1.right_bumper)
            setArmPow(armpower);
        else if (gamepad1.right_trigger > 0)
            setArmPow(-armpower);
        else
            setArmPow(0);

        if (gamepad1.left_bumper)
            setHandPow(handpower);
        else if (gamepad1.left_trigger > 0)
            setHandPow(-handpower);
        else
            setHandPow(0);



        telemetry.addData("Left Pow", left);
        telemetry.addData("Right Pow", right);


        telemetry.addLine()
                .addData("a", colors.alpha )
                .addData("r", (colors.red / (colors.blue + colors.red + colors.green)))
                .addData("g", (colors.green / (colors.blue + colors.red + colors.green)))
                .addData("b", (colors.blue / (colors.blue + colors.red + colors.green)));

        telemetry.update();
    }
}