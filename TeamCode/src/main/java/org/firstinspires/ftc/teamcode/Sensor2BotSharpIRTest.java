package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Hunter Seachrist on 12/5/2017.
 */
@TeleOp(name = "SensorBot: Sharp IR Test", group = "bepis")
@Disabled
public class Sensor2BotSharpIRTest extends Sensor2BotTemplate {
    public void init_loop(){

    }
    boolean thrown;
    public void start(){
        telemetry.addLine("Started");
        telemetry.update();
    }

    public void loop() {
        //telemetry.addLine("Distance from wall (cm): " + ir.readDistanceCM());
        //telemetry.addLine("Voltage" + ir.getSharpIRVoltage());
    }
}
