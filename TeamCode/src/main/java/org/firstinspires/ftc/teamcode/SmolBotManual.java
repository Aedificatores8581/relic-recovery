package org.firstinspires.ftc.teamcode;

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

import java.util.LinkedList;
import java.util.Queue;
import java.util.Timer;
import java.util.TimerTask;

/**
 * I made this from scratch.
 *
 * @author Pootis Man
 */
@TeleOp(name = "RevBot: Tele-Op", group = "bepis")
public class SmolBotManual extends SmolBotTemplate
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
    public SmolBotManual() {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    }

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
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;

        setLeftPow(left);
        setRightPow(right);

        if (gamepad1.a)
            setArmPow(1);
        else if (gamepad1.y)
            setArmPow(-1);
        else
            setArmPow(0);

        if (gamepad1.x)
            setHandPow(1);
        else if (gamepad1.b)
            setHandPow(-1);
        else
            setHandPow(0);

        double srvPos = srv.getPosition();
        srvPos += gamepad1.right_trigger * Constants.SERVO_SPEED;
        srvPos -= gamepad1.left_trigger * Constants.SERVO_SPEED;
        srvPos = Math.max(srvPos, -0.25);
        srvPos = Math.min(srvPos, -0.75);
        srv.setPosition(srvPos);

        telemetry.addData("Left Pow", left);
        telemetry.addData("Right Pow", right);
        telemetry.addData("Srv Pos", srvPos);
    }

} // PootisBotManual
