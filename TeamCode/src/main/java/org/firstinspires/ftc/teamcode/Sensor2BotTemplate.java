package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Const;

/**
 * Conjured into existence by The Saminator on 12-05-2017.
 */
public abstract class Sensor2BotTemplate extends OpMode {
    //protected DigitalChannel touch;
    //protected PingSensor ping;
    protected AnalogInput pingAn;

    protected AnalogInputController pingOn;

    //protected SharpIRSensor ir;
    protected DcMotor lm, rm;
    //protected DigitalChannel magFront, magBack;
    //protected NormalizedColorSensor color;
    //protected NormalizedRGBA colors;
    //protected DistanceSensor dSensor;

    //protected OpticalDistanceSensor ods;


    public static class Constants {
        public static final double MOTOR_POWER = 0.25;
        public static final DcMotor.Direction LM_DIR = DcMotorSimple.Direction.FORWARD;
        public static final DcMotor.Direction RM_DIR = DcMotorSimple.Direction.REVERSE;
    }

    @Override
    public void init() {
        //touch = hardwareMap.digitalChannel.get("touch");

        //pingAn = hardwareMap.get(AnalogInput.class, "ping");

        //pingAn = new AnalogInput()

        //color = hardwareMap.get(NormalizedColorSensor.class, "jcolor");

        //dSensor = hardwareMap.get(DistanceSensor.class, "ds");

        //ping = new PingSensor(hardwareMap.digitalChannel.get("ping"));
        //pingAn = hardwareMap.analogInput.get("ping");
        //ir = new SharpIRSensor(hardwareMap.analogInput.get("ir"));
        //magFront = hardwareMap.digitalChannel.get("mf");
        //magBack = hardwareMap.digitalChannel.get("mb");
        lm = hardwareMap.dcMotor.get("lm");
        rm = hardwareMap.dcMotor.get("rm");

        //touch.setMode(DigitalChannel.Mode.INPUT);
        lm.setDirection(Constants.LM_DIR);
        rm.setDirection(Constants.RM_DIR);
        //magFront.setMode(DigitalChannel.Mode.INPUT);
        //magBack.setMode(DigitalChannel.Mode.INPUT);
    }

    public void go() {
        lm.setPower(Constants.MOTOR_POWER);
        rm.setPower(Constants.MOTOR_POWER);
    }

    public void setLeftPow(double pow){
        lm.setPower(pow);
    }
    public void setRightPow(double pow){
        rm.setPower(pow);
    }

    public void stop() {
        lm.setPower(0.0);
        rm.setPower(0.0);
        //ping.isRunningThread = false;
    }
}
