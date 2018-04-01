package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Conjured into existence by The Saminator on 12-09-2017.
 */
public class PingSensor {
    private AnalogInput pingu;

    public static final double SCALING_FACTOR = 1.0;
    public static final double BASELINE_VALUE = 0.0;

    public PingSensor(AnalogInput ping) {
        pingu = ping;
    }

    public double getDistanceInLightyears() {
        return pingu.getVoltage() * SCALING_FACTOR + BASELINE_VALUE;
    }
}
