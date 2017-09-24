package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

/**
 * Created by Hunter Seachrist on 9/14/2017.
 */
@TeleOp(name = "Detect Blue: Color Sensor Test", group = "Team 8581")
public class DetectBlueColorSensorTest extends ColorSensorBotTemplate{

    @Override public void loop(){

        if (cs.blue() > cs.red() && cs.blue() >= 15){
            setRightPow(0.0);
            setLeftPow(1.0);
        } else if (cs.blue() < cs.red() && cs.red() >= 15){
            setRightPow(1.0);
            setLeftPow(0.0);
        }
        telemetry.addData("Blue: ", cs.blue());
        telemetry.addData("Red: ", cs.red());
        telemetry.addData("Green: ", cs.green());
        telemetry.addData("Alpha: ", cs.alpha());

    }

}
