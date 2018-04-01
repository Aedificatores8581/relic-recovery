package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Mister-Minister-Master on 11/5/2017.
 */

@TeleOp(name = "Non Continuous Servo Test", group = "8581")

@Disabled
public class NonContinuousRotationalServoTest extends OpMode {
    Servo servo;
    double INCREMENT = 0.005;     // amount to slew servo each CYCLE_MS cyclE
    double MAX_POS = .75;     // Maximum rotational position
    double MIN_POS = .25;     // Minimum rotational position

    double servoPos;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_y < 0) {
            // Keep stepping up until we hit the max value.
            servoPos += INCREMENT;
            if (servoPos > MAX_POS) {
                servoPos = MAX_POS;
            }
            servo.setPosition(servoPos);
        }
        if (gamepad1.left_stick_y > 0) {
            servoPos -= INCREMENT;
            if (servoPos < MIN_POS) {
                servoPos = MIN_POS;
            }
            servo.setPosition(servoPos);
        }
        telemetry.addData("Position: ", servo.getPosition());
    }
}
