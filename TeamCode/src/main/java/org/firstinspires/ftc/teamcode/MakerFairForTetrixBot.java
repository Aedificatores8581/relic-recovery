package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by fgpor on 11/11/2017.
 */

@Autonomous(name = "Maker Fair Teleop (30 second version)", group = "super-bepis")

@Disabled
public class MakerFairForTetrixBot extends OpMode {
    DcMotor left, right;

    public void init() {
        left = hardwareMap.dcMotor.get("lm");
        right = hardwareMap.dcMotor.get("rm");
    }

    public void loop() {
        if (Math.abs(gamepad1.left_stick_y) > .5) {
            setLeftPow(gamepad1.left_stick_y);
        } else {
            setLeftPow(0.0);
        }

        if (Math.abs(gamepad1.right_stick_y) > .5) {
            setRightPow(-gamepad1.right_stick_y);
        } else {
            setRightPow(0.0);
        }
    }


    private void setLeftPow(double pow) {
        left.setPower(pow * Constants.LEFT_SPEED);
    }

    private void setRightPow(double pow) {
        right.setPower(pow * Constants.RIGHT_SPEED);
    }
}
