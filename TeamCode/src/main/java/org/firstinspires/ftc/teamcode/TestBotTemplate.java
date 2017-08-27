package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Mister-Minister-Master on 8/27/2017.
 * Basically just SmolBotTemplate with no arm motor or hand motor
 */





public abstract class TestBotTemplate extends OpMode{
    DcMotor left, right;

    @Override
    public void init(){
        left = hardwareMap.dcMotor.get("lm");
        right = hardwareMap.dcMotor.get("rm");

        left.setDirection(Constants.LEFT_DIR);
        right.setDirection(Constants.RIGHT_DIR);

    }

    protected void setLeftPow(double pow) {
        left.setPower(pow * Constants.LEFT_SPEED);
    }

    protected void setRightPow(double pow) {
        right.setPower(pow * Constants.RIGHT_SPEED);
    }


    protected boolean checkEncoder(int ticks) {
        int distance = Math.abs(ticks);
        int leftDist = Math.abs(left.getCurrentPosition());
        int rightDist = Math.abs(right.getCurrentPosition());

        return (distance <= leftDist) || (distance <= rightDist);
    }

}
