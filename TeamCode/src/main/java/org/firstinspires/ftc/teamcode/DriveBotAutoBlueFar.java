package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/*
 * Conjured into existence by The Saminator on 11-12-2017.
 */
@Autonomous(name = "Autonomous Blue Far", group = "competition bepis")

public class DriveBotAutoBlueFar extends DriveBotTestTemplate {

    State state;

    private int cameraMonitorViewId;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;
    private VuforiaLocalizer vuforia;
    private RelicRecoveryVuMark vuMark;

    boolean ranServoInit;
    boolean initServos, sensing = false;
    Gamepad prev1;

    double distanceToWall = 0.35, distanceToCenterOfWall = 0.8;
    double mult = 1;
    long waitTime = 2000L;
    long prevTime, prevTime2 = 0, totalTime = 0;
    double jewelArmDownPosition = 0.74, speed = -0.09, adjustSpeed = 0.06, dispensePosition = 1.0, retractDispensePosition = 0.3;

    //210 to move forward to left
    //325 to move to mid
    //400 to move to right
    //350 to place glyph


    /*
    990 to 1138 for dismount
    130 to right
    483 to 550 to center
    267 to move to front
     */

    int encToDriveToWall = 180, encToDriveToNext = 105;

    int timeToDispense, encToMeetCryptobox = 110, encToDispense = 75, encToRamGlyph = 250, encToBackUp = 150, encToBackUpAgain = 360, encToDismount = 1050;
    double glyphHold = 0.03, glyphDrop = 0.33;
    double ramLeftMod = 1.0, ramRightMod = 1.0, ramAngle = AutonomousDefaults.RAM_MOTOR_RATIO;

    int encToAlignLeft = 888, encToAlignCenter = 450, encToAlignRight = 165;

    double degrees90 = 80;

    CryptoboxColumn column;
    GyroAngles gyroAngles;
    boolean dispenseGlyph, retractDispenser;
    boolean checkKey, keyChecked;

    boolean wallDetected = false;

    int count1 = 0;
    int count = 0;

    // IMPORTANT: THIS OP-MODE WAITS ONE SECOND BEFORE STARTING. THIS MEANS THAT WE HAVE TWENTY-NINE SECONDS TO ACCOMPLISH TASKS, NOT THIRTY.
    public void start() {
        super.start();
        relicTrackables.activate();
        //encToMoveToCenter = encToMoveToLeft + encToChangeColumn;
        //encToMoveToRight = encToMoveToLeft + (encToChangeColumn * 2);
        if (ramAngle > 1.0) {
            ramRightMod = 1.0;
            ramLeftMod = 1.0 / ramAngle;
        } else {
            ramRightMod = ramAngle;
            ramLeftMod = 1.0;
        }

        checkKey = false;
        keyChecked = false;

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


        initServos = true;
        ranServoInit = false;
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

        state = State.STATE_LOWER_JEWEL_ARM;

        rIntake.setPosition(0.3);

        lIntake.setPosition(0.7);

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VuforiaLicenseKey.LICENSE_KEY; // VuforiaLicenseKey is ignored by git
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");

        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        prevTime = 0;

        dispenseGlyph = false;
        retractDispenser = false;

        leftFore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        prev1 = new Gamepad();
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
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

        if (triggered(gamepad1.left_trigger) && !triggered(prev1.left_trigger))

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

        if (gamepad1.right_stick_button && !prev1.right_stick_button)
            encToDismount += 5;

        if (gamepad1.left_stick_button && !prev1.left_stick_button)
            encToDismount -= 5;

        if (triggered(gamepad1.right_stick_x) && !triggered(prev1.right_stick_x))
            ramAngle += 0.05;

        if (triggered(-gamepad1.right_stick_x) && !triggered(-prev1.right_stick_x))
            ramAngle -= 0.05;

        telemetry.addData("Driving Speed (DPad up/down)", speed);
        telemetry.addData("Turning Speed (DPad right/left)", adjustSpeed);
        telemetry.addData("Distance to Dispense Glyph (Right/left bumpers)", encToDispense);
        telemetry.addData("Distance to Back Up First (Left stick up/down)", encToBackUp);
        telemetry.addData("Distance to Ram Glyph (Left stick right/left)", encToRamGlyph);
        telemetry.addData("Distance to Back Up Final (Right stick up/down)", encToBackUpAgain);
        telemetry.addData("Angle to Ram Glyph Second (Right Speed Mult) (Right stick right/left)", ramAngle);
        telemetry.addData("Distance to Dismount Balance (Left/right stick buttons)", ramAngle);

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
        double currentHeading = 0.0;
        boolean hasAngles = false;
        if (angles != null) {
            currentHeading = new GyroAngles(angles).getZ();
            hasAngles = true;
        }

        if (!initServos) {
            initServos = true;

            rIntake.setPosition(0.7);

            lIntake.setPosition(0.3);

            relicHand.setPosition(0.5);
            ranServoInit = true;
        }

        if(System.currentTimeMillis() - totalTime < 500) {
            relicArm.setPower(-1.0);
        }
        else {
            relicArm.setPower(0);
            if (!ranServoInit){
                initServos = false;
            }

        }

        switch (state) {
            case STATE_LOWER_JEWEL_ARM:
                checkKey = true;
                jewelFlipper.setPosition(Constants.CENTER_FINGER);
                jewelArm.setPosition(Constants.JEWEL_ARM_DOWN_POSITION);

                if (prevTime == 0)
                    prevTime = System.currentTimeMillis();
                if (timeReached(prevTime, waitTime))
                    state = State.STATE_SCAN_JEWEL;
                break;
            case STATE_SCAN_JEWEL:

                glyphOutput.setPosition(Constants.LEVEL_DISPENSER);
                jewelFlipper.setPosition(Constants.CENTER_FINGER);
                if(dSensorL.getDistance(DistanceUnit.CM) > dSensorR.getDistance(DistanceUnit.CM) && dSensorL.getDistance(DistanceUnit.CM) - dSensorR.getDistance(DistanceUnit.CM) > 1){
                    setRightPow(0.01);
                    setLeftPow(0.01);
                    mult = 1;
                }
                else if(dSensorL.getDistance(DistanceUnit.CM) < dSensorR.getDistance(DistanceUnit.CM) && dSensorR.getDistance(DistanceUnit.CM) - dSensorL.getDistance(DistanceUnit.CM) > 1){
                    setRightPow(-0.01);
                    setLeftPow(-0.01);
                    mult = -1;
                }
                prevTime = 0;
                if (redRatio > Constants.RED_THRESHOLD)
                    state = State.STATE_HIT_RIGHT_JEWEL;
                else if (redRatio < Constants.RED_THRESHOLD)
                    state = State.STATE_HIT_LEFT_JEWEL;
                else if (timeReached(totalTime, 5000))
                    state = State.STATE_RESET_JEWEL_HITTER;

                break;
            case STATE_HIT_LEFT_JEWEL:
                setLeftPow(.005 * mult);
                setRightPow(0.005 * mult);
                jewelFlipper.setPosition(0.05);
                if(prevTime == 0)
                    prevTime = System.currentTimeMillis();
                if (System.currentTimeMillis() - prevTime >= waitTime)
                    state = State.STATE_RESET_JEWEL_HITTER;
                break;
            case STATE_HIT_RIGHT_JEWEL:
                setLeftPow(.005 * mult);
                setRightPow(0.005 * mult);
                jewelFlipper.setPosition(1.0);
                if(prevTime == 0)
                    prevTime = System.currentTimeMillis();
                if (System.currentTimeMillis() - prevTime >= waitTime)
                    state = State.STATE_RESET_JEWEL_HITTER;
                break;
            case STATE_RESET_JEWEL_HITTER:
                relicHand.setPosition(0.284);
                jewelFlipper.setPosition(Constants.CENTER_FINGER);
                jewelArm.setPosition(Constants.JEWEL_ARM_UP_POSITION);
                state = State.STATE_GYRO_ANGLES;
                break;
            case STATE_SCAN_KEY:
                if (timeReached(prevTime, 10000)) {
                    if (!keyChecked)
                        column = CryptoboxColumn.MID;
                    state = State.STATE_GYRO_ANGLES;
                }
                break;
            case STATE_GYRO_ANGLES:
                gyroAngles = new GyroAngles(angles);
                state = State.STATE_L_APPROACH_CRYPTOBOX;
                switch (column) {
                    case LEFT:
                        setLeftPow(speed);
                        setRightPow(speed);
                        count = 1;
                        break;
                    case MID:
                        setLeftPow(speed);
                        setRightPow(speed);
                        count = 2;
                        break;
                    case RIGHT:
                        setLeftPow(speed);
                        setRightPow(speed);
                        count = 3;
                        break;
                }
                break;
            //<editor-fold desc="Left column">
            case STATE_L_APPROACH_CRYPTOBOX:
                if (checkEncoders(encToDismount)) {
                    setLeftPow(adjustSpeed);
                    setRightPow(-adjustSpeed);
                    state = State.STATE_L_TURN_90;
                }
                break;
            case STATE_L_TURN_90:
                if (gyroAngles.getZ() - currentHeading >= degrees90) {
                    gyroAngles = new GyroAngles(angles);
                    resetEncoders();
                    reinitMotors(speed, speed);
                    state = State.STATE_L_ALIGN_TO_CRYPTOBOX;
                }
                break;
            case STATE_APPROACH_WALL:
                jewelFlipper.setPosition(Constants.CENTER_FINGER);
                jewelArm.setPosition(0.5);
                if(dSensorR.getDistance(DistanceUnit.CM) <= 6 && sensing) {
                    jewelArm.setPosition(Constants.JEWEL_ARM_UP_POSITION);
                    resetEncoders();
                    reinitMotors(speed, speed);

                    sensing = false;
                    state = State.STATE_L_ALIGN_TO_CRYPTOBOX;
                }
                break;
            case STATE_L_ALIGN_TO_CRYPTOBOX:

                if(checkEncoders(Constants.ENC_TO_PASS_COLUMN - 20)&& !sensing) {
                    jewelFlipper.setPosition(Constants.CENTER_FINGER);
                    jewelArm.setPosition(0.5);
                    count1++;
                    sensing = true;
                }
                if(dSensorR.getDistance(DistanceUnit.CM) == Double.NaN)
                    wallDetected = false;
                else
                    wallDetected = true;
                if(dSensorL.getDistance(DistanceUnit.CM) >= dSensorR.getDistance(DistanceUnit.CM) && wallDetected == true && sensing) {
                    wallDetected = true;

                    if(count1 == count) {
                        jewelArm.setPosition(Constants.JEWEL_ARM_UP_POSITION);
                        resetEncoders();
                        reinitMotors(-adjustSpeed, adjustSpeed);
                        state = State.STATE_L_TURN_90_BACK;
                    }
                    else {
                        reinitMotors(speed, speed);
                        state = State.STATE_APPROACH_WALL;
                    }

                }

                break;
            case STATE_DETECT_WALL:
                if(checkEncoders(encToDriveToWall)) {
                    jewelFlipper.setPosition(Constants.CENTER_FINGER);
                    if(prevTime == 0)
                        prevTime = System.currentTimeMillis();
                    timeReached(prevTime, 200);
                    jewelArm.setPosition(0.5);
                    state = State.STATE_L_ALIGN_TO_CRYPTOBOX;
                }
                break;
            case STATE_L_TURN_90_BACK:
                if (prevTime2 == 0)
                    prevTime2 = System.currentTimeMillis();
                if (gyroAngles.getZ() - currentHeading <= -degrees90) {
                    resetEncoders();
                    if (System.currentTimeMillis() - prevTime2 >= 1000) {
                        reinitMotors(speed, speed);
                        state = State.STATE_L_MEET_CRYPTOBOX;
                    }
                }
                break;
            case STATE_L_MEET_CRYPTOBOX:
                if (checkEncoders(encToMeetCryptobox)) {

                    resetEncoders();
                    state = State.STATE_REINIT_MOTORS;
                }
                break;
            //</editor-fold>
            /*
             * Things that we need to do:
             * + Fix the angle checks
             * + Dispense the glyph at the end of the final turn.
             */
            //<editor-fold desc="Center column">
            case STATE_C_APPROACH_CRYPTOBOX:
                if (checkEncoders(encToDismount)) {
                    setLeftPow(adjustSpeed);
                    setRightPow(-adjustSpeed);
                    state = State.STATE_C_TURN_90;
                }
                break;
            case STATE_C_TURN_90:
                if (gyroAngles.getZ() - currentHeading >= degrees90) {
                    resetEncoders();
                    reinitMotors(speed, speed);
                    state = State.STATE_C_ALIGN_TO_CRYPTOBOX;
                }
                break;
            case STATE_C_ALIGN_TO_CRYPTOBOX:
                if (checkEncoders(encToAlignCenter)) {
                    prevTime2 = 0; // Reset the timer for the glyph dispense wait.
                    gyroAngles = new GyroAngles(angles);
                    setLeftPow(-adjustSpeed);
                    setRightPow(adjustSpeed);
                    state = State.STATE_C_TURN_90_BACK;
                }
                break;
            case STATE_C_TURN_90_BACK:
                if (prevTime2 == 0)
                    prevTime2 = System.currentTimeMillis();
                if (gyroAngles.getZ() - currentHeading <= -degrees90) {
                    resetEncoders();
                    if (System.currentTimeMillis() - prevTime2 >= 1000) {
                        reinitMotors(speed, speed);
                        state = State.STATE_C_MEET_CRYPTOBOX;
                    }
                }
                break;
            case STATE_C_MEET_CRYPTOBOX:
                if (checkEncoders(encToMeetCryptobox)) {
                    dispenseGlyph = true;
                    gyroAngles = new GyroAngles(angles);
                    resetEncoders();
                    state = State.STATE_REINIT_MOTORS;
                }
                break;
            //</editor-fold>
            //<editor-fold desc="Right column">
            case STATE_R_APPROACH_CRYPTOBOX:
                if (checkEncoders(encToDismount)) {
                    setLeftPow(adjustSpeed);
                    setRightPow(-adjustSpeed);
                    state = State.STATE_R_TURN_A_BIT;
                }
                break;
            case STATE_R_TURN_A_BIT:
                if (gyroAngles.getZ() - currentHeading >= degrees90) {
                    resetEncoders();
                    reinitMotors(speed, speed);
                    state = State.STATE_R_ALIGN_TO_CRYPTOBOX;
                }
                break;
            case STATE_R_ALIGN_TO_CRYPTOBOX:
                if (checkEncoders(encToAlignRight)) {
                    prevTime2 = 0; // Reset the timer for the glyph dispense wait.
                    gyroAngles = new GyroAngles(angles);
                    setLeftPow(-adjustSpeed);
                    setRightPow(adjustSpeed);
                    state = State.STATE_R_TURN_BACK;
                }
                break;
            case STATE_R_TURN_BACK:
                if (prevTime2 == 0)
                    prevTime2 = System.currentTimeMillis();
                if (gyroAngles.getZ() - currentHeading <= -degrees90) {
                    resetEncoders();
                    if (System.currentTimeMillis() - prevTime2 >= 1000) {
                        reinitMotors(speed, speed);
                        state = State.STATE_R_MEET_CRYPTOBOX;
                    }
                }
                break;
            case STATE_R_MEET_CRYPTOBOX:
                if (checkEncoders(encToMeetCryptobox)) {
                    gyroAngles = new GyroAngles(angles);
                    resetEncoders();
                    state = State.STATE_REINIT_MOTORS;
                }
                break;
            //</editor-fold>
            case STATE_REINIT_MOTORS:
                reinitMotors(speed, speed);
                state = State.STATE_DISPENSE_GLYPH;
                break;
            case STATE_DISPENSE_GLYPH:

                glyphOutput.setPosition(dispensePosition);
                if (checkEncoders(encToDispense)) {
                    resetEncoders();
                    reinitMotors(-speed, -speed);
                    state = State.STATE_BACK_UP_TO_RAM_GLYPH;
                }
                break;
            case STATE_BACK_UP_TO_RAM_GLYPH:

                glyphOutput.setPosition(retractDispensePosition);
                if (checkEncoders(encToBackUp)) {
                    resetEncoders();
                    reinitMotors(speed * ramLeftMod, speed * ramRightMod);
                    state = State.STATE_RAM_GLYPH_INTO_BOX;
                }
                break;
            case STATE_RAM_GLYPH_INTO_BOX:
                if (checkEncoders(encToRamGlyph)) {
                    resetEncoders();
                    reinitMotors(-speed * ramLeftMod, -speed * ramRightMod);
                    state = State.STATE_BACK_AWAY_FROM_RAMMED_GLYPH;
                }
                break;
            case STATE_BACK_AWAY_FROM_RAMMED_GLYPH:
                if (checkEncoders(encToBackUpAgain)) {
                    setLeftPow(0);
                    setRightPow(0);
                    state = State.STATE_END;
                }
                break;
            case STATE_END:
                telemetry.addData("Finished", "Very Yes");
                break;
        }

        if (checkKey) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            switch (vuMark) {
                case LEFT:
                    column = CryptoboxColumn.RIGHT;
                    count = 1;
                    break;
                case CENTER:
                    column = CryptoboxColumn.MID;
                    count = 2;
                    break;
                case RIGHT:
                    column = CryptoboxColumn.LEFT;
                    count = 3;
                    break;
            }
            if (vuMark != RelicRecoveryVuMark.UNKNOWN)
                keyChecked = true;
            else
                column = CryptoboxColumn.MID;
        }



        if (dispenseGlyph) {
            if (retractDispenser) {
                glyphOutput.setPosition(retractDispensePosition);
                if (prevTime == 0)
                    prevTime = System.currentTimeMillis();
                if (System.currentTimeMillis() - prevTime >= waitTime)
                    dispenseGlyph = false;
            } else
                glyphOutput.setPosition(dispensePosition);

            if (prevTime == 0)
                prevTime = System.currentTimeMillis();
            if (System.currentTimeMillis() - prevTime >= waitTime)
                retractDispenser = true;
        }



        if (gyroAngles != null)
            telemetry.addData("Last GyroAngles", gyroAngles.getX() + "," + gyroAngles.getY() + "," + gyroAngles.getZ());
        telemetry.addData("State", state.name());
        telemetry.addData("Red Ratio", redRatio);
        telemetry.addData("Blue Ratio", blueRatio);
        telemetry.addData("Red Color", colors.red);
        telemetry.addData("Blue Color", colors.blue);

        telemetry.addData("Total LF Encoder", leftFore.getCurrentPosition());
        telemetry.addData("Total LR Encoder", leftRear.getCurrentPosition());
        telemetry.addData("Total RF Encoder", rightFore.getCurrentPosition());
        telemetry.addData("Total RR Encoder", rightRear.getCurrentPosition());

        //telemetry.addData("Angle", new GyroAngles(angles).getZ()); // IMPORTANT: DO NOT UNCOMMENT THIS CAUSES A NULL POINTER EXCEPTION!

        telemetry.addData("Jewel Arm Position", jewelArm.getPosition());
        telemetry.addData("Jewel Flipper Position", jewelFlipper.getPosition());
        telemetry.addData("Relic Hand Position", relicHand.getPosition());
        telemetry.addData("Relic Fingers Position", relicFingers.getPosition());
        telemetry.addData("Glyph Output Position", glyphOutput.getPosition());
        telemetry.addData("Right Intake Position", rIntake.getPosition());
        telemetry.addData("Left Intake Position", lIntake.getPosition());

        telemetry.addData("Column walls sensed", count1 + 1);
        if (hasAngles && gyroAngles != null)
            telemetry.addData("Remaining Angle", gyroAngles.getZ() - currentHeading);

        if (column != null)
            telemetry.addData("Column", column.name());
    }

    // These states MUST have comments describing what they do, when they end, and what the next state is.
    enum State {
        STATE_LOWER_JEWEL_ARM, // Ends when jewel arm is at certain position. Always -> STATE_SCAN_JEWEL
        STATE_SCAN_JEWEL, // Ends when right jewel color is read. Right jewel == blue -> STATE_HIT_LEFT_JEWEL. Right jewel == red -> STATE_HIT_RIGHT_JEWEL
        STATE_HIT_LEFT_JEWEL, // Ends when servo is at position. Always -> STATE_RESET_JEWEL_HITTER
        STATE_HIT_RIGHT_JEWEL, // Ends when servo is at position. Always -> STATE_RESET_JEWEL_HITTER
        STATE_RESET_JEWEL_HITTER, // Ends when servo is at position. Always -> STATE_SCAN_KEY
        STATE_SCAN_KEY, // Ends when, if the Vuforia was not successful yet, the column is assumed to be center. Always -> STATE_GYRO_ANGLES
        STATE_GYRO_ANGLES, // Ends when gyroAngles variable is set to the current angles. Key { left -> STATE_L_TURN_90; center -> STATE_C_APPROACH_CRYPTOBOX; right -> STATE_R_APPROACH_CRYPTOBOX }

        /* Old stuff
        STATE_FIRST_TURN, // Ends when motor powers are set. Always -> STATE_CHECK_SLOT
        STATE_CRYPTOBOX_RIGHT_SLOT, // Ends when short-range distance sensor reads cryptobox divider. Key == left -> STATE_DISPENSE_GLYPH. Key == center or right -> STATE_CRYPTOBOX_CENTER_SLOT
        STATE_CRYPTOBOX_CENTER_SLOT, // Ends when short-range distance sensor reads cryptobox divider. Key == center -> STATE_DISPENSE_GLYPH. Key == right -> STATE_CRYPTOBOX_LEFT_SLOT
        STATE_CRYPTOBOX_LEFT_SLOT, // Ends when short-range distance sensor reads cryptobox divider. Always -> STATE_RECORD_FACING.
        STATE_RECORD_FACING, // Ends when current orientation is recorded. Always -> STATE_FACE_CRYPTOBOX
        STATE_FACE_CRYPTOBOX, // Ends when gyro is at angle. Always -> STATE_REINIT_MOTORS*/

        // Left column
        STATE_L_APPROACH_CRYPTOBOX, // Ends when the robot has moved a certain distance. Always -> STATE_L_TURN_90
        STATE_L_TURN_90, // Ends when the robot has turned 90 degrees. Always -> STATE_L_ALIGN_TO_CRYPTOBOX
        STATE_L_ALIGN_TO_CRYPTOBOX, // Ends when the robot has moved a certain distance. Always -> STATE_L_TURN_90_BACK,
        STATE_L_TURN_90_BACK, // Ends when the robot has turned 90 degrees back. Always -> STATE_L_ALIGN_TO_CRYPTOBOX
        STATE_L_MEET_CRYPTOBOX, // Ends when the robot has moved a certain distance. Always -> STATE_REINIT_MOTORS

        // Center column

        //IMPOERTANT: AS OF NOW, CENTER STATES ARE NO LONGER IN USE.
        STATE_C_APPROACH_CRYPTOBOX, // Ends when the robot has moved a certain distance. Always -> STATE_C_TURN_90
        STATE_C_TURN_90, // Ends when the robot has turned 90 degrees. Always -> STATE_C_ALIGN_TO_CRYPTOBOX
        STATE_C_ALIGN_TO_CRYPTOBOX, // Ends when the robot has moved a certain distance. Always -> STATE_C_TURN_90_BACK,
        STATE_C_TURN_90_BACK, // Ends when the robot has turned 90 degrees back. Always -> STATE_C_ALIGN_TO_CRYPTOBOX
        STATE_C_MEET_CRYPTOBOX, // Ends when the robot has moved a certain distance. Always -> STATE_REINIT_MOTORS



        STATE_APPROACH_WALL, //Ends when the robot is positioned in a way in which the jewel arm is inbetween the right edge and the center of the cryptobox, always -> STATE_DETECT_WALL
        STATE_DETECT_WALL,
        STATE_RESET,
        // Right column

        //IMPOERTANT: AS OF NOW, RIGHT STATES ARE NO LONGER IN USE.
        STATE_R_APPROACH_CRYPTOBOX, // Ends when the robot has moved a certain distance. Always -> STATE_R_TURN_A_BIT
        STATE_R_TURN_A_BIT, // Ends when the robot has turned 45 degrees. Always -> STATE_R_ALIGN_TO_CRYPTOBOX
        STATE_R_ALIGN_TO_CRYPTOBOX, // Ends when the robot has moved a certain small distance. Always -> STATE_R_TURN_BACK
        STATE_R_TURN_BACK, // Ends when the robot has turned 45 degrees. Always -> STATE_R_MEET_CRYPTOBOX
        STATE_R_MEET_CRYPTOBOX, // Ends when the robot has moved a certain distance. Always -> STATE_REINIT_MOTORS

        STATE_REINIT_MOTORS, // Ends when the motors' mode is RUN_USING_ENCODER. Always -> STATE_DISPENSE_GLYPH
        STATE_DISPENSE_GLYPH, // Ends when glyph is dispensed. Always -> STATE_WAIT_FOR_GLYPH_DISPENSED
        STATE_BACK_UP_TO_RAM_GLYPH, // Ends when motors are at position. Always -> STATE_RAM_GLYPH_INTO_BOX
        STATE_RAM_GLYPH_INTO_BOX, // Ends when motors are at position. Always -> STATE_BACK_AWAY_FROM_RAMMED_GLYPH
        STATE_BACK_AWAY_FROM_RAMMED_GLYPH, // Ends when motors are at position. Always -> STATE_END

        STATE_DRIVE_INTO_PILE,
        STATE_COLLECT_GLYPH,
        STATE_DRIVE_BACK,

        STATE_END // Ends when the universe dies. Always -> STATE_RESURRECT_UNIVERSE



        // STATE_RESURRECT_UNIVERSE // uncomment when we have the technology to reverse entropy.
    }

}
