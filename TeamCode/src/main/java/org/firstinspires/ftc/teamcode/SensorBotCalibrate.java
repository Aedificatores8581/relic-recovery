package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by The Saminator on 08-12-2017.
 */
@Autonomous(name = "SensorBot: Calibrate", group = "feelz")
@Disabled
public class SensorBotCalibrate extends SensorBotTemplate {
    private boolean prevA;

    @Override
    public void init() {
        super.init();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.getAll(BNO055IMU.class).get(0);
        imu.initialize(parameters);
    }

    @Override
    public void loop() {
        if (prevA && !gamepad1.a) {
            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("AdafruitIMUCalibration.json"), imu.readCalibrationData().serialize());
        }
        prevA = gamepad1.a;
    }
}
