package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Hunter Seachrist on 3/17/2018.
 */

@TeleOp(name = "Now that's a Spicy One!", group = "spicy bepis")
public class SpicyDrive extends DriveBotTestTemplate {

    private static final double TURN_MULT = .75;

    @Override
    public void init(){
        super.init();
    }

    @Override
    public void loop(){
        setLeftPow((-gamepad1.left_stick_y) - TURN_MULT * (gamepad1.right_stick_x * gamepad1.left_stick_y));
        setRightPow((-gamepad1.left_stick_y) + TURN_MULT * (gamepad1.right_stick_x * gamepad1.left_stick_y));
    }

}
