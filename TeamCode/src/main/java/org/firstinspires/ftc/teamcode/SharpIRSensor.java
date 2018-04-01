package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Conjured into existence by The Saminator on 12-13-2017.
 */
public class SharpIRSensor {
    private AnalogInput sharpIR;

    private double convertDistanceCM(double value) {
        return 10853.481 * Math.pow(value * 5000 / 1024, -1.2);
    }

    public SharpIRSensor(AnalogInput sharpIR) {
        this.sharpIR = sharpIR;
    }

    public double readDistanceCM() throws Error {
        double voltage = sharpIR.getVoltage();
        if (voltage < 0.0)
            throw new Error("Voltage is negative! SHUT. DOWN. EVERYTHING.");
        return convertDistanceCM(sharpIR.getVoltage());
    }

    public double getSharpIRVoltage(){
        return sharpIR.getVoltage();
    }
}
