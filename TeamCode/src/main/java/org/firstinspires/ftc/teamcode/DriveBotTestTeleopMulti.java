package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Conjured into existence by The Saminator on 10-01-2017.
 */
@TeleOp(name = "DriveBot Multi-thread Tele-Op", group = "this is a test")
public class DriveBotTestTeleopMulti extends DriveBotTestTemplate {
    public class RobotBalanceThread extends Thread {
        @Override
        public void run() {
            try {

                while (!Thread.interrupted()) {
                    if (isBalancing && gravity != null) {
                        Spherical3D gravAngles = cartesianToSpherical(new Cartesian3D(gravity.zAccel, gravity.xAccel, gravity.yAccel));

                        if (gravAngles.theta > 3.375) {
                            double leftPow = -0.25;
                            double rightPow = -0.25;

                            // Account for fore/back tilt
                            double sign = Math.sin(org.firstinspires.ftc.teamcode.Constants.DEGS_TO_RADS * (gravAngles.phi));
                            sign /= Math.abs(sign);
                            double foreBack = sign * Math.sin(org.firstinspires.ftc.teamcode.Constants.DEGS_TO_RADS * (gravAngles.theta) / 2.0);

                            leftPow *= foreBack;
                            rightPow *= foreBack;

                            if (gamepad1.left_stick_x > 0 ^ leftPow < 0) {
                                leftPow *= 1 - (2 * gamepad1.left_stick_x);
                            } else {
                                rightPow *= 1 + (2 * gamepad1.left_stick_x);
                            }

                            setLeftPow(leftPow);
                            setRightPow(rightPow);
                        } else {
                            setLeftPow(0);
                            setRightPow(0);
                            isBalancing = false;
                        }
                    }
                }
            } catch (Throwable t) {
                exception = t;
            }
        }
    }

    public class InputHandlerThread extends Thread {
        @Override
        public void run() {
            boolean moveLifter = false;
            boolean resetLifter = false;

            try {
                while (!Thread.interrupted()) {
                    // Reset encoders
                    if (gamepad1.left_stick_button)
                        inputActions.add(new Runnable() {
                            @Override
                            public void run() {
                                leftFore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                rightFore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                rightFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                leftFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            }
                        });

                    // Move glyph intake and belt
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

                    if ((gamepad2.left_stick_y > 0 && magFront.getState()) || (gamepad2.left_stick_y < 0 && magBack.getState()) || gamepad2.left_stick_y == 0)
                        relicArm.setPower(gamepad2.left_stick_y);

                    // Gear shift
                    if (gamepad1.x && !prev1.x)
                        inputActions.add(new Runnable() {
                            @Override
                            public void run() {
                                toggleSpeed();
                            }
                        });

                    if (gamepad1.b && !prev1.b)
                        isBalancing = !isBalancing;

                    // Move glyph lift
                    if (triggered(gamepad2.left_trigger) && glyphLiftHigh.getState()) {
                        glyphLift.setPower(0.5);
                    } else if (triggered(gamepad2.right_trigger) && glyphLiftLow.getState()) {
                        glyphLift.setPower(-0.5);
                    } else if (!triggered(gamepad2.right_trigger) && !triggered(gamepad2.left_trigger)) {
                        glyphLift.setPower(0.0);
                    }

                    // Toggle auto-glyphing

                    if ((gamepad2.x && !prev2.x) || (gamepad2.y && !prev2.y))
                        inputActions.add(new Runnable() {
                            @Override
                            public void run() {
                                enableAutoGlyph = !enableAutoGlyph;
                            }
                        });

                    if (enableAutoGlyph)
                        if ((gamepad2.a && !prev2.a) || (gamepad2.b && !prev2.b))
                            inputActions.add(new Runnable() {
                                @Override
                                public void run() {
                                    switch (glyphLiftState) {
                                        case LEVELED:
                                            glyphLift.setPower(0.5);
                                            glyphLiftState = GlyphLiftState.ASCENDING;
                                            break;
                                        case ASCENDED:
                                            glyphLiftState = GlyphLiftState.DUMPING;
                                            break;
                                        case DESCENDED:
                                            glyphLiftState = GlyphLiftState.LEVELING;
                                            break;
                                        case DUMPED:
                                            glyphLift.setPower(-0.5);
                                            glyphOutput.setPosition(0.5);
                                            glyphLiftState = GlyphLiftState.DESCENDING;
                                            break;
                                    }

                                    dumpServoManual = false;
                                }
                            });

                    try {
                        prev1.copy(gamepad1);
                        prev2.copy(gamepad2);
                    } catch (RobotCoreException e) {
                        telemetry.addData("Exception", e);
                    }
                }
            } catch (Throwable t) {
                exception = t;
            }
        }
    }

    public class ServoGetterThread extends Thread {
        @Override
        public void run() {
            while (!Thread.interrupted()) {
                getServosFromGamepad();
                refreshServos();
            }
        }
    }

    private Gamepad prev1;
    private Gamepad prev2;

    private Throwable exception;

    private Thread gamepadThread;
    private Queue<Runnable> inputActions;

    private Thread balanceThread;
    private volatile boolean isBalancing = false;

    private Thread getServoThread;

    private GlyphLiftState glyphLiftState;
    private SpeedToggle speedMult;
    private byte armPos = 1;
    private volatile double jewelArmServoValue = 0, jewelFlipperServoValue = 0, relicHandServoValue = 0, relicFingersServoValue = 0, glyphDumpServoValue = 0;
    private boolean lifting, valueChange;
    private boolean armExtended;
    private boolean enableAutoGlyph = false;
    private boolean dumpServoManual;
    long waiting = 0, waitTime = 500;

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

    public enum GlyphLiftState {
        LEVELING(true),
        LEVELED(false),
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
        return true;
    }

    @Override
    public void init() { // Configuration for this is in the Google Drive
        super.init();
        prev1 = new Gamepad();
        prev2 = new Gamepad();
        armExtended = false;

        glyphLiftState = GlyphLiftState.DESCENDED;

        inputActions = new ConcurrentLinkedQueue<>();

        gamepadThread = new InputHandlerThread();
        balanceThread = new RobotBalanceThread();
        getServoThread = new ServoGetterThread();
    }

    @Override
    public void start() {
        super.start();

        jewelArmServoValue = 0;
        jewelFlipperServoValue = 0.5;
        relicFingersServoValue = 0.5;
        speedMult = SpeedToggle.SLOW;
        jewelFlipper.setPosition(0.5);
        relicHand.setPosition(0.4);
        glyphOutput.setPosition(0.0);

        dumpServoManual = true;
        enableAutoGlyph = false;

        isBalancing = false;

        exception = null;

        gamepadThread.start();
        balanceThread.start();
        getServoThread.start();
    }

    @Override
    public void stop() {
        super.stop();

        gamepadThread.interrupt();
        balanceThread.interrupt();
        getServoThread.interrupt();
    }

    protected void toggleSpeed() {
        if (speedMult.equals(SpeedToggle.SLOW))
            speedMult = SpeedToggle.FAST;
        else
            speedMult = SpeedToggle.SLOW;
    }

    protected void getServosFromGamepad() {
        // Move jewel arm down
        if (gamepad1.dpad_down) {
            jewelArmServoValue -= 0.01;
        }

        // Move jewel arm up
        if (gamepad1.dpad_up) {
            jewelArmServoValue += 0.01;
        }

        // Move glyph dumper up
        if (gamepad2.dpad_up || gamepad2.y) {
            glyphDumpServoValue += 0.05;
            dumpServoManual = true;
        }

        // Move glyph dumper down
        if (gamepad2.dpad_down || gamepad2.a) {
            glyphDumpServoValue -= 0.05;
            dumpServoManual = true;
        }

        // Move glyph dumper level
        if (gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.x) {
            glyphDumpServoValue = 0.42;
            dumpServoManual = true;
        }

        // Move jewel flipper
        if (gamepad1.dpad_left) {
            jewelFlipperServoValue += 0.01;
        }

        // Move jewel flipper
        if (gamepad1.dpad_right) {
            jewelFlipperServoValue -= 0.01;
        }

        // Move relic hand
        if (Math.abs(gamepad2.right_stick_y) >= 0.25) {
            relicHandServoValue += gamepad2.right_stick_y * 0.012;
        }

        // Move relic finger in
        if (gamepad2.right_bumper) {
            relicFingersServoValue -= 0.02;
        }

        // Move relic finger out
        if (gamepad2.left_bumper) {
            relicFingersServoValue += 0.02;
        }
    }

    protected void refreshServos() {
        jewelArm.setPosition(Utilities.clamp(0.25, jewelArmServoValue, 0.8));
        jewelFlipper.setPosition(Utilities.clamp(0.05, jewelFlipperServoValue, 0.95));
        relicHand.setPosition(Utilities.clamp(0, relicHandServoValue, 1));
        relicFingers.setPosition(Utilities.clamp(0.23, relicFingersServoValue, 0.63));
        if (relicFingersServoValue > 0.63) // Maximum position
            relicFingersServoValue = 0.63;
        if (relicFingersServoValue < 0.23) // Minimum position
            relicFingersServoValue = 0.23;
        if (dumpServoManual)
            glyphOutput.setPosition(Utilities.clamp(0, glyphDumpServoValue, 1));
    }

    protected void setMotorPowers() {
        setLeftPow(gamepad1.left_stick_y * -speedMult.getMult());
        setRightPow(gamepad1.right_stick_y * -speedMult.getMult());
    }

    @Override
    public void loop() {
        try {
            if (!isBalancing)
                setMotorPowers();

            while (!inputActions.isEmpty()) {
                Runnable action = inputActions.poll();
                if (action != null)
                    action.run();
            }

            if (exception != null)
                throw exception;

            switch (glyphLiftState) {
                case LEVELING:
                    glyphOutput.setPosition(0.42);
                    glyphLiftState = GlyphLiftState.LEVELED;
                    break;
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
                        glyphOutput.setPosition(0.0);
                        glyphLiftState = GlyphLiftState.DESCENDED;
                    }
                    break;
            }

            telemetry.addData("Glyph lift state", glyphLiftState);
            telemetry.addData("Relic Fingers", relicFingersServoValue);

            telemetry.addData("Relic Fingers pos ", relicFingers.getPosition());

            try {
                prev1.copy(gamepad1);
                prev2.copy(gamepad2);
            } catch (RobotCoreException e) {
                telemetry.addData("Exception", e);
            }
        } catch (Throwable e) {
            exception = null;
            gamepadThread.interrupt();
            balanceThread.interrupt();
            getServoThread.interrupt();
            telemetry.addData("Exception", e);
            //throw e;
        }
    }
}
