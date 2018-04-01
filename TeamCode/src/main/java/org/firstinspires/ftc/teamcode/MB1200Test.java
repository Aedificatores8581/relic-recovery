package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Conjured into existence by The Saminator on 02-01-2018.
 */

@Disabled
@Autonomous(name = "MB1200 Test", group = "these are not the opmodes you are looking for move along now")
public class MB1200Test extends OpMode {
    private MB1200 sensor;

    @Override
    public void init() {
        sensor = new MB1200(hardwareMap.digitalChannel.get("mb"));
    }

    @Override
    public void start() {
        super.start();
        sensor.start();
    }

    @Override
    public void loop() {
        if (sensor.hasReading())
            telemetry.addData("Sensor reading", sensor.getReading());
        telemetry.addData("Sensor state", sensor.getState());
    }

    @Override
    public void stop() {
        super.stop();
        sensor.stop();
    }
}
