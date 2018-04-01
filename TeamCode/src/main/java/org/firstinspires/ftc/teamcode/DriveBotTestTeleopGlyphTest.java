package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Conjured into existence by The Saminator on 10-01-2017.
 */

@Disabled
@TeleOp(name = "DriveBot Glyph Test Teleop", group = "this is a test")
public class DriveBotTestTeleopGlyphTest extends DriveBotTestTemplate {
    private Gamepad prev1;
    private Gamepad prev2;

    private GlyphLiftState glyphLiftState;
    private SpeedToggle speedMult;
    private byte armPos = 1;
    private double jewelArmServoValue = 0, jewelFlipperServoValue = 0, relicHandServoValue = 0, relicFingersServoValue = 0, glyphDumpServoValue = 0;
    private boolean lifting, valueChange;
    private boolean armExtended;
    long waiting = 0, waitTime = 500;

    public enum SpeedToggle {
        SLOW(0.45),
        FAST(.8);

        private double mult;

        SpeedToggle(double mult) {
            this.mult = mult;
        }

        public double getMult() {
            return mult;
        }
    }

    public enum GlyphLiftState {
        ASCENDING(true),
        ASCENDED(false),
        DUMPING(true),
        DUMPED(false),
        DESCENDING(true),
        DESCENDED(false);

        private boolean isMoving;

        GlyphLiftState(boolean moving) {
            isMoving = moving;
        }

        public boolean currentlyMoving() {
            return isMoving;
        }
    }

    @Override
    protected boolean needsGyroSensor() {
        return false;
    }

    @Override
    public void init() { // Configuration for this is in the Google Drive
        super.init();
        prev1 = new Gamepad();
        prev2 = new Gamepad();
        armExtended = false;

        glyphLiftState = GlyphLiftState.DESCENDED;
    }

    @Override
    public void start() {
        jewelArmServoValue = 0;
        jewelFlipperServoValue = 0.5;
        relicFingersServoValue = 0.5;
        speedMult = SpeedToggle.SLOW;
        jewelFlipper.setPosition(0.5);
        relicHand.setPosition(0.4);
        glyphOutput.setPosition(0.0);
    }

    protected void toggleSpeed() {
        if (speedMult.equals(SpeedToggle.SLOW))
            speedMult = SpeedToggle.FAST;
        else
            speedMult = SpeedToggle.SLOW;
    }

    protected void clampJewelArmServo() {
        if (jewelArmServoValue > 0.8) // Maximum position
            jewelArmServoValue = 0.8;
        if (jewelArmServoValue < 0.25) // Minimum position
            jewelArmServoValue = 0.25;
    }

    protected void clampJewelFlipperServo() {
        if (jewelFlipperServoValue > 0.95)
            jewelFlipperServoValue = 0.95;
        if (jewelFlipperServoValue < 0.05)
            jewelFlipperServoValue = 0.05;
    }

    protected void clampRelicHandServo() {

        if (relicHandServoValue > 1) // Maximum position
            relicHandServoValue = 1;
        if (relicHandServoValue < 0) // Minimum position
            relicHandServoValue = 0;
        //0.188 = 0
        //0.23 = 270
        /*
        arm position of servos
        relic hand 270 degrees = 0.25
        relic hand 0 degrees = 0.2
        relic hand start position = 0.165
         */
    }

    protected void clampRelicFingersServo() {
        if (relicFingersServoValue > 1) // Maximum position
            relicFingersServoValue = 1;
        if (relicFingersServoValue < 0) // Minimum position
            relicFingersServoValue = 0;
    }

    protected void clampDumpServo() {
        if (glyphDumpServoValue > 1) // Maximum position
            glyphDumpServoValue = 1;
        if (glyphDumpServoValue < 0) // Minimum position
            glyphDumpServoValue = 0;
    }

    protected void refreshServos() {
        clampJewelArmServo();
        clampJewelFlipperServo();
        clampRelicHandServo();
        clampRelicFingersServo();
        clampDumpServo();

        jewelArm.setPosition(jewelArmServoValue);
        jewelFlipper.setPosition(jewelFlipperServoValue);
        relicHand.setPosition(relicHandServoValue);
        relicFingers.setPosition(relicFingersServoValue);

        if (!glyphLiftState.currentlyMoving())
            glyphOutput.setPosition(glyphDumpServoValue);
    }

    protected void setMotorPowers() {
        setLeftPow(gamepad1.left_stick_y * -speedMult.getMult());
        setRightPow(gamepad1.right_stick_y * -speedMult.getMult());
    }

    @Override
    public void loop() {
        //if (isDancing())
        //    dance();
        //else
        setMotorPowers();
        refreshServos();

        relicArm.setPower(gamepad2.left_stick_y);

        if (gamepad1.left_stick_button) {
            leftFore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (triggered(gamepad1.right_trigger)) {
            succ(-1.0);
            belt(0.5);
        } else if (triggered(gamepad1.left_trigger)) {
            succ(1.0);
            belt(-0.5);
        } else {
            succ(0.0);
            belt(0.0);
        }

        //if (gamepad1.a && !prev1.a)
        //    scream();

        if (gamepad1.x && !prev1.x)
            toggleSpeed();

        if (gamepad1.dpad_down) {
            jewelArmServoValue -= 0.01;
            clampJewelArmServo();
        }

        if (gamepad1.dpad_up) {
            jewelArmServoValue += 0.01;
            clampJewelArmServo();
        }

        if (gamepad2.dpad_up || gamepad2.y) {
            glyphDumpServoValue += 0.05;
            clampDumpServo();
        }

        if (gamepad2.dpad_down || gamepad2.a) {
            glyphDumpServoValue -= 0.05;
            clampDumpServo();
        }

        if (gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.x) {
            glyphDumpServoValue = 0.5;
            clampDumpServo();
        }

        if (gamepad1.dpad_left) {
            jewelFlipperServoValue += 0.01;
            clampJewelArmServo();
        }

        if (gamepad1.dpad_right) {
            jewelFlipperServoValue -= 0.01;
            clampJewelFlipperServo();
        }

        if (Math.abs(gamepad2.right_stick_y) >= 0.25) {
            relicHandServoValue += gamepad2.right_stick_y * 0.012;
            clampRelicHandServo();
        }

        //if (gamepad2.a && !prev2.a)
        //    startDance();

        //if (gamepad2.b && !prev2.b)
        //    stopDance();

        //if (gamepad2.b) {
        //    relicHandServoValue = 0.188;
        //}

        //increase servo range of relic hand servo
        // program the new servos and motor
/*
        if (gamepad2.a) {
            relicHandServoValue = 0.23;
            if (waiting == 0)
                waiting = System.currentTimeMillis();
            if (System.currentTimeMillis() - waiting >= waitTime) {
                waiting = 0;
                relicFingersServoValue = 0.9;
            }
        }

        if (gamepad1.b) {
            double angleValue = new GyroAngles(angles).getZ() - angleAtStart;
            if (angleValue > new GyroAngles(angles).getZ()) {
                setLeftPow(-0.1);
                setRightPow(0.1);
                if (angleValue <= 45) {
                    setLeftPow(0);
                    setRightPow(0);
                }
            } else if (angleValue < new GyroAngles(angles).getZ()) {
                setLeftPow(0.1);
                setRightPow(-0.1);
                if (angleValue >= 45) {
                    setLeftPow(0);
                    setRightPow(0);
                }
            }
        }
        */
        //850 encoder ticks to get off of the platform (600)
        //320 for right column
        //260 to enter
        //600 to turn
        /*if(gamepad1.x) {
            double angleValue = new GyroAngles(angles).getZ() - angleAtStart;
            if (angleValue > new GyroAngles(angles).getZ()) {
                setLeftPow(-0.1);
                setRightPow(0.1);
                if (angleValue <= 135) {
                    setLeftPow(0);
                    setRightPow(0);
                }
            } else if (angleValue < new GyroAngles(angles).getZ()) {
                setLeftPow(0.1);
                setRightPow(-0.1);
                if (angleValue >= 135) {
                    setLeftPow(0);
                    setRightPow(0);
                }
            }
        }
        */
/*
        if(gamepad2.a){
            while(valueChange = true) {
                if (armPos > 1)
                    armPos--;
                valueChange = false;
            }
        }
        if(gamepad2.b) {
            while (valueChange = true) {
                if (armPos < 3)
                    armPos++;
                valueChange = false;
            }
        }

        if(armPos == 1)
            relicHandServoValue = 0.5;
        if(armPos == 2)
            relicHandServoValue = 0.36;
        if(armPos == 3)
            relicHandServoValue = 0.0;
*/
        //clampRelicHandServo();
        //down = 0.36
        //0.3 up
        //0.5

        if (gamepad2.right_bumper) {
            relicFingersServoValue -= 0.02;
            clampRelicFingersServo();
        }

        if (gamepad2.left_bumper) {
            relicFingersServoValue += 0.02;
            clampRelicFingersServo();
        }

        if (triggered(gamepad2.left_trigger) && glyphLiftHigh.getState())
            glyphLift.setPower(0.75);
        else if (triggered(gamepad2.right_trigger) && glyphLiftLow.getState())
            glyphLift.setPower(-0.75);
        else
            glyphLift.setPower(0.0);

        /*if (gamepad2.x && !prev2.x)
            switch (glyphLiftState) {
                case ASCENDED:
                    glyphLiftState = GlyphLiftState.DUMPING;
                    break;
                case DESCENDED:
                    glyphLift.setPower(0.1);
                    glyphLiftState = GlyphLiftState.ASCENDING;
                    break;
                case DUMPED:
                    glyphLift.setPower(-0.1);
                    glyphOutput.setPosition(0.0);
                    glyphLiftState = GlyphLiftState.DESCENDING;
                    break;
            }

        switch (glyphLiftState) {
            case ASCENDING:
                if (!glyphLiftHigh.getState()) {
                    glyphLift.setPower(0);
                    glyphLiftState = GlyphLiftState.ASCENDED;
                }
                break;
            case DUMPING:
                glyphOutput.setPosition(1);
                glyphLiftState = GlyphLiftState.DUMPED;
                break;
            case DESCENDING:
                if (!glyphLiftLow.getState()) {
                    glyphLift.setPower(0);
                    glyphLiftState = GlyphLiftState.DESCENDED;
                }
                break;
        }*/

        /*telemetry.addData("Arm Extended", armExtended);

        telemetry.addData("Left front power", leftFore.getPower());
        telemetry.addData("Left back power", leftRear.getPower());
        telemetry.addData("Right front power", rightFore.getPower());
        telemetry.addData("Right back power", rightRear.getPower());

        telemetry.addData("Left front encoder", leftFore.getCurrentPosition());
        telemetry.addData("Left back encoder", leftRear.getCurrentPosition());
        telemetry.addData("Right front encoder", rightFore.getCurrentPosition());
        telemetry.addData("Right back encoder", rightRear.getCurrentPosition());

        telemetry.addData("Jewel Arm Pos.", jewelArm.getPosition());
        telemetry.addData("Jewel Arm Set Value", jewelArmServoValue);

        telemetry.addData("Jewel Flip. Pos.", jewelFlipper.getPosition());
        telemetry.addData("Jewel Flip. Set Value", jewelFlipperServoValue);

        telemetry.addData("Relic Hand Pos.", relicHand.getPosition());
        telemetry.addData("Relic Hand Set Value", relicHandServoValue);

        telemetry.addData("Relic Fingers Pos.", relicFingers.getPosition());
        telemetry.addData("Relic Fingers Set Value", relicFingersServoValue);

        telemetry.addData("Glyph Output Pos.", glyphOutput.getPosition());
        telemetry.addData("Glyph Output Set Value", glyphDumpServoValue);

        telemetry.addData("Glyph Lift Upper Sensor", !glyphLiftHigh.getState());
        telemetry.addData("Glyph Lift Lower Sensor", !glyphLiftLow.getState());

        NormalizedRGBA colors = color.getNormalizedColors();
        telemetry.addData("Color Sensor RGB", "[" + colors.red + "," + colors.green + "," + colors.blue + "]");

        telemetry.addData("Speed", speedMult.getMult());*/

        try {
            prev1.copy(gamepad1);
            prev2.copy(gamepad2);
        } catch (RobotCoreException e) {
            telemetry.addData("Exception", e);
        }
    }
}
