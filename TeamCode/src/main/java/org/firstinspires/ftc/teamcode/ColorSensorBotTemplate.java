package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Mister-Minister-Master on 9/14/2017.
 */

public abstract class ColorSensorBotTemplate extends OpMode{
    DcMotor left, right;
    ColorSensor cs;

    protected void setLeftPow(double pow) {
        left.setPower(pow * Constants.LEFT_SPEED);
    }
    protected void setRightPow(double pow) {
        right.setPower(pow * Constants.RIGHT_SPEED);
    }

    public void init(){
        left = hardwareMap.dcMotor.get("lm");
        right = hardwareMap.dcMotor.get("rm");

        cs = hardwareMap.colorSensor.get("cs");
    }

    public void stop(){
        setLeftPow(0.0);
        setRightPow(0.0);
    }

}
