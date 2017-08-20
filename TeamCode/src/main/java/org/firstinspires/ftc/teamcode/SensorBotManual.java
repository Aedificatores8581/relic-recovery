package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------------------
//
// PootisBotManual
//

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * I made this from scratch.
 *
 * @author Pootis Man
 */
@TeleOp(name = "SensorBot: Tele-Op", group = "feelz")
public class SensorBotManual extends SensorBotTemplate {


    //--------------------------------------------------------------------------
    //
    //
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------
    public SensorBotManual() {
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
        color.enableLed(true);
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
        double left = gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;

        setLeftPow(left);
        setRightPow(right);

        //telemetry.addData("Rotation (Yaw,Roll,Pitch)", navx.getYaw() + ", " + navx.getRoll() + ", " + navx.getPitch());
        //telemetry.addData("Gyroscope (XYZ)", navx.getRawGyroX() + ", " + navx.getRawGyroY() + ", " + navx.getRawGyroZ());
        //telemetry.addData("Accelerometer (XYZ)", navx.getRawAccelX() + ", " + navx.getRawAccelY() + ", " + navx.getRawAccelZ());
        //telemetry.addData("Quaternion (WXYZ)", navx.getQuaternionW() + ", " + navx.getQuaternionX() + ", " + navx.getQuaternionY() + ", " + navx.getQuaternionZ());
        telemetry.addData("Red", color.red());
        telemetry.addData("Green", color.green());
        telemetry.addData("Blue", color.blue());
    }

} // PootisBotManual
