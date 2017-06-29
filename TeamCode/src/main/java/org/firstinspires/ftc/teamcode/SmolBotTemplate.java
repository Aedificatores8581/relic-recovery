package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by The Saminator on 06-29-2017.
 */
public abstract class SmolBotTemplate extends OpMode {
    DcMotor leftFore, rightFore, leftRear, rightRear;

    @Override
    public void init() {
        leftFore = hardwareMap.dcMotor.get("lfm");
        leftRear = hardwareMap.dcMotor.get("lbm");
        rightFore = hardwareMap.dcMotor.get("rfm");
        rightRear = hardwareMap.dcMotor.get("rbm");

        leftFore.setDirection(Constants.LEFT_FORE_DIR);
        leftRear.setDirection(Constants.LEFT_REAR_DIR);
        rightFore.setDirection(Constants.RIGHT_FORE_DIR);
        rightRear.setDirection(Constants.RIGHT_REAR_DIR);
    }

    @Override
    public void stop() {
        setLeftPow(0.0);
        setRightPow(0.0);
    }

    protected void setLeftPow(double pow) {
        leftFore.setPower(pow * Constants.LEFT_FORE_SPEED);
        leftRear.setPower(pow * Constants.LEFT_REAR_SPEED);
    }

    protected void setRightPow(double pow) {
        rightFore.setPower(pow * Constants.RIGHT_FORE_SPEED);
        rightRear.setPower(pow * Constants.RIGHT_REAR_SPEED);
    }

    protected boolean checkEncoder(int ticks) {
        int distance = Math.abs(ticks);
        int leftForeDist = Math.abs(leftFore.getCurrentPosition());
        int rightForeDist = Math.abs(rightFore.getCurrentPosition());
        int leftRearDist = Math.abs(leftRear.getCurrentPosition());
        int rightRearDist = Math.abs(rightRear.getCurrentPosition());

        return (distance <= leftForeDist) || (distance <= rightForeDist) || (distance <= leftRearDist) || (distance <= rightRearDist);
    }
}
