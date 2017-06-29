package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------------------
//
// PootisBotManual
//

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * I made this from scratch.
 *
 * @author Pootis Man
 */
@Autonomous(name = "Smolbot: Not Tele-Op", group = "bepis")
public class SmolBotNotManual extends OpMode {
    DcMotor left, right;
    Servo srv;
    long lastServoChange;


    //--------------------------------------------------------------------------
    //
    // PootisBotManual
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------
    public SmolBotNotManual() {
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
        srv = hardwareMap.servo.get("srv");
    }

    @Override
    public void start() {
        srv.setPosition(0.5);
        lastServoChange = System.currentTimeMillis();
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
    @Override public void loop() {
        if (System.currentTimeMillis() - lastServoChange >= 1000) {
            srv.setPosition(srv.getPosition() * -1);
            lastServoChange = System.currentTimeMillis();
        }

        double lpow = Math.sin(System.currentTimeMillis() / 500.0) * 0.5;
        double rpow = Math.cos(System.currentTimeMillis() / 500.0) * 0.5;
        left.setPower(lpow);
        right.setPower(rpow);
        telemetry.addData("L. Motor Power", lpow);
        telemetry.addData("R. Motor Power", rpow);
        telemetry.addData("Time since last servo change", System.currentTimeMillis() - lastServoChange);
        telemetry.addData("Servo Position", srv.getPosition());
    } // PootisBotManual::loop

    @Override public void stop() {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

} // PootisBotManual
