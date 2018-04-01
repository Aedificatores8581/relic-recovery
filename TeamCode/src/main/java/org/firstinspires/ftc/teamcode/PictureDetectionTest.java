package org.firstinspires.ftc.teamcode;

/*
 * Created by Mister-Minister-Master on 11/1/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/* This program detects one of the three VuMark Picture, and based on the picture, will move a set distance.
 *      1. If pictureState is set to PictureState.RIGHT, then it will move (X + a constant) encoder ticks
 *      2. If the same variable is set to PictureState.MID, then it will move (X + 2 times a constant)encoder ticks
 *      3. If the same variable is set to PicutreState.LEFT, then it will move (X + 3 times a constant)encoder ticks
 */

@TeleOp(name = "SensorBot: Vuforia Test", group = "pro-bepis")
@Disabled
public class PictureDetectionTest extends OpMode {

    private VuforiaLocalizer vuforia;
    private DcMotor left, right;
    private int encoderAmount;
    private final int ONE_SECOND = 1000;

    private final int ENCODER_CONSTANT = 500;

    private int cameraMonitorViewId;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    private RelicRecoveryVuMark vuMark;

    private enum RobotActivityState {READING, MOVING}

    private RobotActivityState state;

    @Override
    public void init() {
        state = RobotActivityState.READING;

        left = hardwareMap.dcMotor.get("lm");
        right = hardwareMap.dcMotor.get("rm");

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //parameters.vuforiaLicenseKey = VuforiaLicenseKey.LICENSE_KEY; // VuforiaLicenseKey is ignored by git
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

    }

    public void start() {
        relicTrackables.activate();

        telemetry.addData("activated", "");

        try {
            Thread.sleep(ONE_SECOND);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void loop() {
        if (state == RobotActivityState.READING) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            switch (vuMark) {
                case LEFT:
                    state = RobotActivityState.MOVING;
                    encoderAmount = ENCODER_CONSTANT;
                    break;
                case CENTER:
                    state = RobotActivityState.MOVING;
                    encoderAmount = ENCODER_CONSTANT * 2;
                    break;
                case RIGHT:
                    state = RobotActivityState.MOVING;
                    encoderAmount = ENCODER_CONSTANT * 3;
                    break;
                default:
                    state = RobotActivityState.READING;
                    break;
            }
        } else if (state == RobotActivityState.MOVING) {
            if (!checkEncoder(encoderAmount)) {
                setLeftPow(.5);
                setRightPow(-.5);
            } else {
                setLeftPow(0.0);
                setRightPow(0.0);
            }
        }
        telemetry.addData("Left", left.getCurrentPosition());
        telemetry.addData("Right", right.getCurrentPosition());
    }

    public void stop() {
        setLeftPow(0.0);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setRightPow(0.0);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setLeftPow(double pow) {
        left.setPower(pow * Constants.LEFT_SPEED);
    }

    private void setRightPow(double pow) {
        right.setPower(pow * Constants.RIGHT_SPEED);
    }

    private boolean checkEncoder(int ticks) {
        int distance = Math.abs(ticks);
        int leftDist = Math.abs(left.getCurrentPosition());
        int rightDist = Math.abs(right.getCurrentPosition());

        return (distance <= leftDist) || (distance <= rightDist);
    }

}
