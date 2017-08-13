package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by The Saminator on 06-29-2017.
 */
public abstract class SmolBotTemplate extends OpMode {
    DcMotor left, right, arm, hand;
    Servo srv;

    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("lm");
        right = hardwareMap.dcMotor.get("rm");
        arm = hardwareMap.dcMotor.get("am");
        hand = hardwareMap.dcMotor.get("hm");

        left.setDirection(Constants.LEFT_DIR);
        right.setDirection(Constants.RIGHT_DIR);
        arm.setDirection(Constants.ARM_DIR);
        hand.setDirection(Constants.HAND_DIR);

        srv = hardwareMap.servo.get("srv");

        srv.setPosition(0.0);
    }

    @Override
    public void stop() {
        setLeftPow(0.0);
        setRightPow(0.0);
    }

    protected void setLeftPow(double pow) {
        left.setPower(pow * Constants.LEFT_SPEED);
    }

    protected void setRightPow(double pow) {
        right.setPower(pow * Constants.RIGHT_SPEED);
    }

    protected void setArmPow(double pow) {
        arm.setPower(pow * Constants.ARM_SPEED);
    }

    protected void setHandPow(double pow) {
        hand.setPower(pow * Constants.HAND_SPEED);
    }

    protected boolean checkEncoder(int ticks) {
        int distance = Math.abs(ticks);
        int leftDist = Math.abs(left.getCurrentPosition());
        int rightDist = Math.abs(right.getCurrentPosition());

        return (distance <= leftDist) || (distance <= rightDist);
    }
}
