package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Conjured into existence by The Saminator on 12-05-2017.
 */

@TeleOp(name = "Sensor TeleOp", group = "feelz2thesequel")
@Disabled
public class Sensor2BotTeleop extends Sensor2BotTemplate {
//    @Override
//    public void start() {

//   }

    @Override public void loop() {
/*
        telemetry.addData("Ping raw return values 0:   ", pingAn.getAnalogInputVoltage(0));
        telemetry.addData("Ping raw return values 1:   ", pingAn.getAnalogInputVoltage(1));
        telemetry.addData("Ping raw return values 2:   ", pingAn.getAnalogInputVoltage(2));
        telemetry.addData("Ping raw return values 3:   ", pingAn.getAnalogInputVoltage(3));
*/
        //NormalizedRGBA colors = color.getNormalizedColors();

        //telemetry.addData("Distance (cm)",
        //        String.format(Locale.US, "%.02f", dSensor.getDistance(DistanceUnit.CM)));

        //telemetry.addData("color distance withA ", colors.red + colors.green + colors.blue + colors.alpha);

        //telemetry.addData("color distance       ", colors.red + colors.green + colors.blue);

        //telemetry.addData("color distance    r  ", colors.red);

        //telemetry.addData("color distance    g  ", colors.green);

        //telemetry.addData("color distance    b  ", colors.blue);

        //telemetry.addData("color distance    a  ", colors.alpha);


        /*telemetry.addData("back.getState:  ", magBack.getState());
        telemetry.addData("back.getMode:   ", magBack.getMode());
        telemetry.addData("front.getState:  ", magFront.getState());
        telemetry.addData("front.getMode:   ", magFront.getMode());*/
/*        double relicHandPos = relicHand.getPosition();
//        double relicFingersPos = relicFingers.getPosition();
//
//        if (gamepad1.right_stick_y <= 0 || magnetSensor.getState())
//            relicArm.setPower(gamepad1.right_stick_y);
//
//        if (triggered(Math.abs(gamepad1.left_stick_y))
//          relicHandPos += gamepad1.left_stick_y * Constants.RELIC_HAND_DELTA_MULT;
//
//        if (triggered(gamepad1.right_trigger))
//            relicFingersPos += Constants.RELIC_FINGERS_DELTA;
/        if (triggered(gamepad1.left_trigger))
            relicFingersPos -= Constants.RELIC_FINGERS_DELTA;

        relicHand.setPosition(relicHandPos);
        relicFingers.setPosition(relicFingersPos);

        telemetry.addData("Magnet detected", !magnetSensor.getState());
        telemetry.addData("Hand position", relicHandPos);
        telemetry.addData("Fingers position", relicFingersPos);
        */
    }

}