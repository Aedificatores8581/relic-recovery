package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Conjured into existence by The Saminator on 03-05-2018.
 */
@Autonomous(name = "Simple Ping Test (config 'ps')", group = "succ")
@Disabled
public class SimplePingTest extends OpMode {
    private PingSensor distance;

    @Override
    public void init() {
        distance = new PingSensor(hardwareMap.analogInput.get("ps"));
    }

    @Override
    public void loop() {
        telemetry.addData("Distance (LY)", distance.getDistanceInLightyears());
    }
}
