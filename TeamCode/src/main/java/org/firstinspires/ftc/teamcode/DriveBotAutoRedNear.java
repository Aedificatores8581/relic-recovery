package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/*
 * Conjured into existence by The Saminator on 11-12-2017.
 */
@Autonomous(name = "Autonomous Red Near", group = "competition bepis")

public class DriveBotAutoRedNear extends DriveBotTestTemplate {

    public enum CryptoboxColumn {
        RIGHT("R"),
        MID("C"),
        LEFT("L");

        private String name;

        CryptoboxColumn(String n) {
            name = n;
        }

        public String toString() {
            return name;
        }
    }

    State state;

    boolean sensing = false;
    private int cameraMonitorViewId;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;
    private VuforiaLocalizer vuforia;
    private RelicRecoveryVuMark vuMark;

    boolean initServos;

    boolean wallDetected = false;
    int mult = -1;
    Gamepad prev1;
    int count = 2;
    int count1 = 0;
    long waitTime = 1600L;
    long prevTime, totalTime = 0;
    double speed = 0.075, adjustSpeed = 0.06, dispensePosition = 1.0, retractDispensePosition = 0.0;

    //355  675 1050
    int timeToDispense, encToDispense = 485, encToRamGlyph = 400, encToBackUp = 350, encToBackUpAgain = 300, encToMoveToLeft = /*1130*/325, encToMoveToCenter = /*1530*/690, encToMoveToRight = /*1885*/1000;
    double glyphHold = 0.03, glyphDrop = 0.33;
    double targetAngle = 77;
    double ramLeftMod, ramRightMod, ramAngle = AutonomousDefaults.RAM_MOTOR_RATIO;
    CryptoboxColumn column;
    GyroAngles gyroAngles;
    boolean dispenseGlyph, retractDispenser, checkKey, keyChecked;
    JewelDirection direction;
    JewelColor jewelColor;


    // IMPORTANT: THIS OP-MODE WAITS ONE SECOND BEFORE STARTING. THIS MEANS THAT WE HAVE TWENTY-NINE SECONDS TO ACCOMPLISH TASKS, NOT THIRTY.
    public void start() {
        super.start();

        if (ramAngle > 1.0) {
            ramRightMod = 1.0;
            ramLeftMod = 1.0 / ramAngle;
        } else {
            ramRightMod = ramAngle;
            ramLeftMod = 1.0;
        }

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            telemetry.addData("Exception", e);
        }

        leftFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rIntake.setPosition(0.3);

        lIntake.setPosition(0.7);

        initServos = false;

        direction = null;

        jewelColor = null;
        totalTime = System.currentTimeMillis();


    }

    @Override
    protected boolean needsGyroSensor() {
        return true;
    }

    @Override
    public void init() {
        this.msStuckDetectInit = 10000;
        super.init();

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VuforiaLicenseKey.LICENSE_KEY; // VuforiaLicenseKey is ignored by git
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();
        state = State.STATE_LOWER_JEWEL_ARM;


        rIntake.setPosition(0.3);

        lIntake.setPosition(0.7);

        prevTime = 0;

        dispenseGlyph = false;
        retractDispenser = false;
        checkKey = false;
        keyChecked = false;

        leftFore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        prev1 = new Gamepad();
        column = CryptoboxColumn.MID;
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        switch (vuMark) { // Blue is weird.
            case LEFT:
                column = CryptoboxColumn.RIGHT;
                count = 3;
                break;
            case RIGHT:
                column = CryptoboxColumn.LEFT;
                count = 2;
                break;
            case CENTER:
                column = CryptoboxColumn.MID;
                count = 1;
                break;

        }
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up && !prev1.dpad_up)
            speed += 0.01;

        if (gamepad1.dpad_down && !prev1.dpad_down)
            speed -= 0.01;

        if (gamepad1.dpad_right && !prev1.dpad_right)
            adjustSpeed += 0.005;

        if (gamepad1.dpad_left && !prev1.dpad_left)
            adjustSpeed -= 0.005;

        if (triggered(gamepad1.right_trigger) && !triggered(prev1.right_trigger))
            targetAngle += 1;

        if (triggered(gamepad1.left_trigger) && !triggered(prev1.left_trigger))
            targetAngle -= 1;

        if (gamepad1.a && !prev1.a)
            encToMoveToLeft += 10;

        if (gamepad1.b && !prev1.b)
            encToMoveToLeft -= 10;

        if (gamepad1.left_stick_button && !prev1.left_stick_button)
            encToMoveToCenter += 10;

        if (gamepad1.right_stick_button && !prev1.right_stick_button)
            encToMoveToCenter -= 10;

        if (gamepad1.x && !prev1.x)
            encToMoveToRight += 10;

        if (gamepad1.y && !prev1.y)
            encToMoveToRight -= 10;

        if (gamepad1.right_bumper && !prev1.right_bumper)
            encToDispense += 5;

        if (gamepad1.left_bumper && !prev1.left_bumper)
            encToDispense -= 5;

        if (triggered(gamepad1.left_stick_y) && !triggered(prev1.left_stick_y))
            encToBackUp += 5;

        if (triggered(-gamepad1.left_stick_y) && !triggered(-prev1.left_stick_y))
            encToBackUp -= 5;

        if (triggered(gamepad1.left_stick_x) && !triggered(prev1.left_stick_x))
            encToRamGlyph += 5;

        if (triggered(-gamepad1.left_stick_x) && !triggered(-prev1.left_stick_x))
            encToRamGlyph -= 5;

        if (triggered(gamepad1.right_stick_y) && !triggered(prev1.right_stick_y))
            encToBackUpAgain += 5;

        if (triggered(-gamepad1.right_stick_y) && !triggered(-prev1.right_stick_y))
            encToBackUpAgain -= 5;

        if (triggered(gamepad1.right_stick_x) && !triggered(prev1.right_stick_x))
            ramAngle += 0.05;

        if (triggered(-gamepad1.right_stick_x) && !triggered(-prev1.right_stick_x))
            ramAngle -= 0.05;

        telemetry.addData("Driving Speed (DPad up/down)", speed);
        telemetry.addData("Turning Speed (DPad right/left)", adjustSpeed);
        telemetry.addData("Target Angle Degrees (Right/left triggers)", targetAngle);
        telemetry.addData("Distance to Nearest Cryptobox Column (A/B)", encToMoveToLeft);
        telemetry.addData("Distance to Middle Cryptobox Column (Press on Left/Right Stick)", encToMoveToCenter);
        telemetry.addData("Distance to Farthest Cryptobox Column (X/Y)", encToMoveToRight);
        telemetry.addData("Distance to Dispense Glyph (Right/left bumpers)", encToDispense);
        telemetry.addData("Distance to Back Up First (Left stick up/down)", encToBackUp);
        telemetry.addData("Distance to Ram Glyph (Left stick right/left)", encToRamGlyph);
        telemetry.addData("Distance to Back Up Final (Right stick up/down)", encToBackUpAgain);
        telemetry.addData("Angle to Ram Glyph Second (Right Speed Mult) (Right stick right/left)", ramAngle);

        try {
            prev1.copy(gamepad1);
        } catch (RobotCoreException e) {
            telemetry.addData("Exception", e);
        }
    }

    @Override
    public void loop() {
        NormalizedRGBA colors = color.getNormalizedColors();
        double redRatio = colors.red / (colors.red + colors.green + colors.blue);
        double blueRatio = colors.blue / (colors.red + colors.green + colors.blue);
        if(System.currentTimeMillis() - totalTime < 500)
            relicArm.setPower(-1.0);
        else
            relicArm.setPower(0);

        if (!initServos) {
            initServos = true;

            rIntake.setPosition(0.7);

            lIntake.setPosition(0.3);
        }

        switch (state) {
            case STATE_LOWER_JEWEL_ARM:
                setLeftPow(-0.005);
                setRightPow(-0.005);
                belt(0.5);
                checkKey = true;
                jewelFlipper.setPosition(Constants.CENTER_FINGER);
                jewelArm.setPosition(Constants.JEWEL_ARM_DOWN_POSITION);
                if (prevTime == 0)
                    prevTime = System.currentTimeMillis();
                if (System.currentTimeMillis() - prevTime >= waitTime)
                    state = State.STATE_SCAN_JEWEL;
                break;
            case STATE_SCAN_JEWEL:

                belt(0);
                if(dSensorL.getDistance(DistanceUnit.CM) > dSensorR.getDistance(DistanceUnit.CM) && dSensorL.getDistance(DistanceUnit.CM) - dSensorR.getDistance(DistanceUnit.CM) > 1){
                    setRightPow(-0.01);
                    setLeftPow(-0.01);
                    mult = -1;
                }
                else if(dSensorL.getDistance(DistanceUnit.CM) < dSensorR.getDistance(DistanceUnit.CM) && dSensorR.getDistance(DistanceUnit.CM) - dSensorL.getDistance(DistanceUnit.CM) > 1){
                    setRightPow(0.01);
                    setLeftPow(0.01);
                    mult = 1;
                }
                glyphOutput.setPosition(Constants.LEVEL_DISPENSER);
                prevTime = 0;
                jewelFlipper.setPosition(Constants.CENTER_FINGER);
                if (redRatio > Constants.RED_THRESHOLD)
                    state = State.STATE_HIT_LEFT_JEWEL;
                else if (redRatio < Constants.RED_THRESHOLD)
                    state = State.STATE_HIT_RIGHT_JEWEL;
                else if (System.currentTimeMillis() - totalTime >= 6000)
                    state = State.STATE_RESET_JEWEL_HITTER;
                break;
            case STATE_HIT_LEFT_JEWEL:
                setLeftPow(.005 * mult);
                setRightPow(0.005 * mult);
                jewelFlipper.setPosition(0.05);
                if (prevTime == 0)
                    prevTime = System.currentTimeMillis();
                if (System.currentTimeMillis() - prevTime >= waitTime)
                    state = State.STATE_RESET_JEWEL_HITTER;
                break;
            case STATE_HIT_RIGHT_JEWEL:
                setLeftPow(.005 * mult);
                setRightPow(0.005 * mult);
                jewelFlipper.setPosition(1.0);
                if (prevTime == 0)
                    prevTime = System.currentTimeMillis();
                if (System.currentTimeMillis() - prevTime >= waitTime)
                    state = State.STATE_RESET_JEWEL_HITTER;
                break;
            case STATE_RESET_JEWEL_HITTER:
                relicHand.setPosition(0.284);
                jewelFlipper.setPosition(Constants.CENTER_FINGER);
                jewelArm.setPosition(Constants.JEWEL_ARM_DETECT_POSITION);
                sensing = false;
                setLeftPow(speed);
                setRightPow(speed);
                sensing = true;
                state = state.STATE_DRIVE_TO_CRYPTOBOX;
                break;
            case STATE_DRIVE_TO_CRYPTOBOX:
                jewelFlipper.setPosition(Constants.CENTER_FINGER);
                jewelArm.setPosition(Constants.JEWEL_ARM_DETECT_POSITION);
                if(dSensorL.getDistance(DistanceUnit.CM) <= 6 && sensing) {
                    jewelArm.setPosition(Constants.JEWEL_ARM_UP_POSITION);
                    resetEncoders();
                    reinitMotors(speed, speed);

                    sensing = false;
                    state = State.STATE_CRYPTOBOX_RIGHT_SLOT;
                }
                break;
            case STATE_CRYPTOBOX_RIGHT_SLOT:

                if(checkEncoders(Constants.ENC_TO_PASS_COLUMN - 20)&& !sensing) {
                    jewelFlipper.setPosition(Constants.CENTER_FINGER);
                    jewelArm.setPosition(Constants.JEWEL_ARM_DETECT_POSITION);
                    count1++;
                    sensing = true;
                }
                if(dSensorL.getDistance(DistanceUnit.CM) == Double.NaN)
                    wallDetected = false;
                else
                    wallDetected = true;
                if(dSensorL.getDistance(DistanceUnit.CM) <= dSensorR.getDistance(DistanceUnit.CM) && wallDetected == true && sensing) {
                    if(count1 == count) {
                        jewelArm.setPosition(Constants.JEWEL_ARM_UP_POSITION);
                        resetEncoders();
                        reinitMotors(-adjustSpeed, adjustSpeed);
                        state = State.STATE_RECORD_FACING;
                    }
                    else {
                        reinitMotors(speed, speed);
                        state = State.STATE_DRIVE_TO_CRYPTOBOX;
                    }

                }
                break;
            case STATE_CRYPTOBOX_CENTER_SLOT:
                if (checkEncoders(encToMoveToCenter)) {
                    if (column == CryptoboxColumn.MID)
                        state = State.STATE_RECORD_FACING;
                    else
                        state = State.STATE_CRYPTOBOX_LEFT_SLOT;
                }
                break;
            case STATE_CRYPTOBOX_LEFT_SLOT:
                if (checkEncoders(encToMoveToRight)) {
                    state = State.STATE_RECORD_FACING;
                }
                break;
            case STATE_RECORD_FACING:
                gyroAngles = new GyroAngles(angles);
                state = State.STATE_FACE_CRYPTOBOX;
                break;
            case STATE_FACE_CRYPTOBOX:
                setLeftPow(-adjustSpeed);
                setRightPow(adjustSpeed);
                if (gyroAngles.getZ() - (new GyroAngles(angles).getZ()) <= -targetAngle) {
                    resetEncoders();
                    state = State.STATE_REINIT_MOTORS;
                }
                break;
            case STATE_REINIT_MOTORS:
                reinitMotors(-speed, -speed);
                state = State.STATE_DISPENSE_GLYPH;
                break;
            case STATE_DISPENSE_GLYPH:
                if (checkEncoders(encToDispense)) {
                    dispenseGlyph = true;
                    resetEncoders();
                    reinitMotors(speed, speed);
                    state = State.STATE_BACK_UP_TO_RAM_GLYPH;
                }
                break;
            case STATE_BACK_UP_TO_RAM_GLYPH:
                if (checkEncoders(encToBackUp)) {
                    resetEncoders();
                    reinitMotors(-speed * ramLeftMod, -speed * ramRightMod);
                    state = State.STATE_RAM_GLYPH_INTO_BOX;
                }
                break;
            case STATE_RAM_GLYPH_INTO_BOX:
                if (checkEncoders(encToRamGlyph)) {
                    resetEncoders();
                    reinitMotors(speed * ramLeftMod, speed * ramRightMod);
                    state = State.STATE_BACK_AWAY_FROM_RAMMED_GLYPH;
                }
                break;
            case STATE_BACK_AWAY_FROM_RAMMED_GLYPH:
                if (checkEncoders(encToBackUpAgain)) {
                    setLeftPow(0);
                    setRightPow(0);
                    glyphOutput.setPosition(retractDispensePosition);
                    state = State.STATE_END;
                }
                break;
            case STATE_END:
                telemetry.addData("Finished", "Very Yes");
                break;
        }

        if(System.currentTimeMillis() - totalTime < 500)
            relicArm.setPower(-1.0);
        else
            relicArm.setPower(0);

        if (dispenseGlyph) {
            glyphOutput.setPosition(dispensePosition);
            if (prevTime == 0)
                prevTime = System.currentTimeMillis();
            if (System.currentTimeMillis() - prevTime >= waitTime)
                retractDispenser = true;

            if (retractDispenser) {

                //glyphOutput.setPosition(retractDispensePosition);
                if (prevTime == 0)
                    prevTime = System.currentTimeMillis();
                if (System.currentTimeMillis() - prevTime >= waitTime)
                    dispenseGlyph = false;

            }

        }

        if (checkKey) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            switch (vuMark) {
                case LEFT:
                    column = CryptoboxColumn.LEFT;
                    count = 3;
                    break;
                case RIGHT:
                    column = CryptoboxColumn.RIGHT;
                    count = 1;
                    break;
                case CENTER:
                    column = CryptoboxColumn.MID;
                    count = 2;
                    break;
            }
            if (vuMark != RelicRecoveryVuMark.UNKNOWN)
                keyChecked = true;
        }


        telemetry.addData("DETECTED WALL", wallDetected);

        telemetry.addData("count", count);
        telemetry.addData("count", count1);

        telemetry.addData("State", state.name());
        telemetry.addData("Red Ratio", redRatio);
        telemetry.addData("Blue Ratio", blueRatio);
        telemetry.addData("Red Color", colors.red);
        telemetry.addData("Blue Color", colors.blue);

        telemetry.addData("Total LF Encoder", leftFore.getCurrentPosition());
        telemetry.addData("Total LR Encoder", leftRear.getCurrentPosition());
        telemetry.addData("Total RF Encoder", rightFore.getCurrentPosition());
        telemetry.addData("Total RR Encoder", rightRear.getCurrentPosition());

        telemetry.addData("Jewel Arm Position", jewelArm.getPosition());
        telemetry.addData("Jewel Flipper Position", jewelFlipper.getPosition());
        telemetry.addData("Relic Hand Position", relicHand.getPosition());
        telemetry.addData("Relic Fingers Position", relicFingers.getPosition());
        telemetry.addData("Glyph Output Position", glyphOutput.getPosition());
        telemetry.addData("Right Intake Position", rIntake.getPosition());
        telemetry.addData("Left Intake Position", lIntake.getPosition());

        telemetry.addData("Checking Cryptobox Key", checkKey);
        telemetry.addData("Cryptobox Key Checked", keyChecked);

        if (direction != null)
            telemetry.addData("Jewel Direction", direction.name());

        if (jewelColor != null)
            telemetry.addData("Jewel Color", jewelColor.name());

        if (column != null)
            telemetry.addData("Cryptobox Column", column.toString());



        telemetry.addData("dR  ", runWithArmDistance(dSensorR));

        telemetry.addData("dL  ", runWithArmDistance(dSensorL));

        telemetry.addData("dL  ", dSensorL.getDistance(DistanceUnit.CM));

        telemetry.addData("dR  ", dSensorR.getDistance(DistanceUnit.CM));
    }

    enum State {
        STATE_LOWER_JEWEL_ARM, // Ends when jewel arm is at certain position. Always -> STATE_SCAN_JEWEL
        STATE_SCAN_JEWEL, // Ends when right jewel color is read. Right jewel == blue -> STATE_HIT_LEFT_JEWEL. Right jewel == red -> STATE_HIT_RIGHT_JEWEL
        STATE_HIT_LEFT_JEWEL, // Ends when servo is at position. Always -> STATE_RESET_JEWEL_HITTER
        STATE_HIT_RIGHT_JEWEL, // Ends when servo is at position. Always -> STATE_RESET_JEWEL_HITTER
        STATE_RESET_JEWEL_HITTER, // Ends when servo is at position. Always -> STATE_DRIVE_TO_CRYPTOBOX
        STATE_DRIVE_TO_CRYPTOBOX, // Ends when short-range distance sensor reads cryptobox divider. Always -> STATE_CRYPTOBOX_RIGHT_SLOT
        STATE_CRYPTOBOX_RIGHT_SLOT, // Ends when short-range distance sensor reads cryptobox divider. Key == left -> STATE_DISPENSE_GLYPH. Key == center or right -> STATE_CRYPTOBOX_CENTER_SLOT
        STATE_CRYPTOBOX_CENTER_SLOT, // Ends when short-range distance sensor reads cryptobox divider. Key == center -> STATE_DISPENSE_GLYPH. Key == right -> STATE_CRYPTOBOX_LEFT_SLOT
        STATE_CRYPTOBOX_LEFT_SLOT, // Ends when short-range distance sensor reads cryptobox divider. Always -> STATE_RECORD_FACING.
        STATE_RECORD_FACING, // Ends when current orientation is recorded. Always -> STATE_FACE_CRYPTOBOX
        STATE_FACE_CRYPTOBOX, // Ends when gyro is at angle. Always -> STATE_REINIT_MOTORS
        STATE_REINIT_MOTORS, // Ends when the motors' mode is RUN_USING_ENCODER. Always -> STATE_DISPENSE_GLYPH
        STATE_DISPENSE_GLYPH, // Ends when glyph is dispensed. Always -> STATE_WAIT_FOR_GLYPH_DISPENSED
        STATE_BACK_UP_TO_RAM_GLYPH, // Ends when motors are at position. Always -> STATE_RAM_GLYPH_INTO_BOX
        STATE_RAM_GLYPH_INTO_BOX, // Ends when motors are at position. Always -> STATE_BACK_AWAY_FROM_RAMMED_GLYPH
        STATE_BACK_AWAY_FROM_RAMMED_GLYPH, // Ends when motors are at position. Always -> STATE_END
        STATE_DRIVE_TO_PILE,
        STATE_INTAKE,
        STATE_DRIVE_BACK,
        STATE_END // Ends when the universe dies. Always -> STATE_RESURRECT_UNIVERSE
        // STATE_RESURRECT_UNIVERSE // uncomment when we have the technology to reverse entropy.
    }

}


//left - 1195 to 1265
//center - 1575 to 1620
//right - 1967a