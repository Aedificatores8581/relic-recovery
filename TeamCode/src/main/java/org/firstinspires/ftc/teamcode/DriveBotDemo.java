package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.DriveBotTestTemplate;

import java.util.ArrayList;

@TeleOp(name = "Drive Bot: One Controller", group = "Drive Bot")
public class DriveBotDemo extends DriveBotTestTemplate {

    enum ControlState{
        GAMEPAD_ONE,
        GAMEPAD_TWO
    }

    public enum SpeedToggle {
        SLOW(0.6), // originally 0.45
        FAST(0.7); // originally 0.80

        private double mult;

        SpeedToggle(double mult) {
            this.mult = mult;
        }

        public double getMult() {
            return mult;
        }
    }


    ControlState controlState;
    SpeedToggle speedMult;

    double relicHandServoValue;
    double relicFingersServoValue;
    double glyphDumpServoValue;

    Gamepad prev;

    @Override
    public void init() {
        super.init();
        prev = new Gamepad();
        controlState = ControlState.GAMEPAD_ONE;
    }




    public void start() {
        relicFingersServoValue = 0.5;
        speedMult = SpeedToggle.SLOW;
        jewelFlipper.setPosition(0.5);
        relicHand.setPosition(0.4);
        glyphOutput.setPosition(0.0);
        jewelArm.setPosition(Constants.JEWEL_ARM_UP_POSITION);
    }




    @Override
    public void loop() {
        switch (controlState){

            case GAMEPAD_ONE:
                if (triggered(gamepad1.right_trigger)) {
                    succ(1.0);
                    belt(0.5);
                } else if (triggered(gamepad1.left_trigger)) {
                    succ(-1.0);
                    belt(-0.5);
                } else {
                    succ(0.0);
                    belt(0.0);
                }

                if (gamepad1.left_bumper && glyphLiftHigh.getState())
                    glyphLift.setPower(0.75);
                else if (gamepad1.right_bumper && glyphLiftLow.getState())
                    glyphLift.setPower(-0.75);
                else
                    glyphLift.setPower(0.0);

                setLeftPow(gamepad1.left_stick_y * -speedMult.getMult());
                setRightPow(gamepad1.right_stick_y * -speedMult.getMult());
                break;




            case GAMEPAD_TWO:
                relicArm.setPower(gamepad1.left_stick_y);
                if (Math.abs(gamepad1.right_stick_y) >= 0.25) {
                    relicHandServoValue += gamepad1.right_stick_y * 0.012;
                    clampRelicHandServo();
                }
                if (gamepad1.right_bumper) {
                    relicFingersServoValue -= 0.02;
                    clampRelicFingersServo();
                }

                if (gamepad1.left_bumper) {
                    relicFingersServoValue += 0.02;
                    clampRelicFingersServo();
                }

                break;
        }




        if (gamepad1.dpad_up) {
            glyphDumpServoValue += 0.05;
            clampDumpServo();
        }

        if (gamepad1.dpad_down) {
            glyphDumpServoValue -= 0.05;
            clampDumpServo();
        }

        if (gamepad1.dpad_left || gamepad1.dpad_right) {
            glyphDumpServoValue = Constants.LEVEL_DISPENSER;
            clampDumpServo();
        }




        if (gamepad1.x && !prev.x){
            toggleSpeed();
        }

        if (gamepad1.a && !prev.a) {
            controlState = toggleControlState(controlState);
        }

        try {
            prev.copy(gamepad1);
        } catch (RobotCoreException e) {
            telemetry.addData("Exception", e);
        }

        jewelArm.setPosition(Constants.JEWEL_ARM_UP_POSITION);
        refreshServos();
    }






    protected void toggleSpeed() {
        if (speedMult.equals(SpeedToggle.SLOW))
            speedMult = SpeedToggle.FAST;
        else
            speedMult = SpeedToggle.SLOW;
    }
    protected ControlState toggleControlState(ControlState controlState) {
        return (controlState == ControlState.GAMEPAD_ONE)
                ? ControlState.GAMEPAD_TWO : ControlState.GAMEPAD_ONE;
    }


    // Keeps RelicHandServoValue within the range 0-1
    protected void clampRelicHandServo() {

        if (relicHandServoValue > 1) // Maximum position
            relicHandServoValue = 1;
        if (relicHandServoValue < 0) // Minimum position
            relicHandServoValue = 0;
    }

    protected void clampRelicFingersServo() {
        if (relicFingersServoValue > 0.65) // Maximum position
            relicFingersServoValue = 0.65;
        if (relicFingersServoValue < 0.23) // Minimum position
            relicFingersServoValue = 0.23;
    }

    protected void clampDumpServo() {
        if (glyphDumpServoValue > 1) // Maximum position
            glyphDumpServoValue = 1;
        if (glyphDumpServoValue < 0) // Minimum position
            glyphDumpServoValue = 0;
    }

    protected void refreshServos() {
        clampRelicHandServo();
        clampRelicFingersServo();
        clampDumpServo();

        relicHand.setPosition(relicHandServoValue);
        relicFingers.setPosition(relicFingersServoValue);
        glyphOutput.setPosition(glyphDumpServoValue);

    }

    protected void setMotorPowers() {
        setLeftPow(gamepad1.left_stick_y * -speedMult.getMult());
        setRightPow(gamepad1.right_stick_y * -speedMult.getMult());
    }



}