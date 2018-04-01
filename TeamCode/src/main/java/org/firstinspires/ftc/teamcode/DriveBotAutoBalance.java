package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.Locale;
/**
 * Created by fgpor on 3/3/2018.
 */

@Disabled
public class DriveBotAutoBalance extends DriveBotTestTemplate{

    final double CLOSE = 0.0;
    final double POW_MULTIPLYER = 1.0;
    final double POW_MULT_2 = 0;
    boolean fail = false;
    public void start(){
        super.start();
        leftFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void init(){
        super.init();
    }
    @Override
    public void loop(){
        NormalizedRGBA colorsR = color.getNormalizedColors();
        NormalizedRGBA colorsL = colorL.getNormalizedColors();

        double rD = csDistance(colorsR) * 10, lD = csDistance(colorsL) * 10;
        double relOr = rD - lD;
        if(csDistance (colorsR) >= CLOSE || csDistance (colors) >= CLOSE || fail == true) {
            jewelArm.setPosition(Constants.JEWEL_ARM_UP_POSITION);
            fail = true;
        }
        else
            jewelArm.setPosition(Constants.JEWEL_ARM_DOWN_POSITION);
        setLeftPow(relOr * POW_MULTIPLYER);
        setRightPow(relOr * POW_MULTIPLYER);
        long time = System.currentTimeMillis();
        if(timeReached(time, 10)) {
            setLeftPow(relOr );
            setRightPow(relOr * POW_MULTIPLYER);
        }
    }

}
