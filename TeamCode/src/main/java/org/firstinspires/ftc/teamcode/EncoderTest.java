package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/*
 * Conjured into existence by The Saminator on 11-12-2017.
 * ..
 */

@Disabled
@Autonomous(name = "Encoder Test", group = "test bepis")
public class EncoderTest extends OpMode {

    private static class Constants {
        public static final DcMotor.Direction LEFT_FORE_DIR = DcMotor.Direction.REVERSE;
        public static final DcMotor.Direction LEFT_REAR_DIR = DcMotor.Direction.REVERSE;
        public static final DcMotor.Direction RIGHT_FORE_DIR = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction RIGHT_REAR_DIR = DcMotor.Direction.FORWARD;

        public static final double LEFT_FORE_SPEED = 0.375;
        public static final double LEFT_REAR_SPEED = 0.375;
        public static final double RIGHT_FORE_SPEED = 0.375;
        public static final double RIGHT_REAR_SPEED = 0.375;
    }

    protected void setLeftPow(double pow) {
        leftFore.setPower(pow * Constants.LEFT_FORE_SPEED);
        leftRear.setPower(pow * Constants.LEFT_REAR_SPEED);
    }

    protected void setRightPow(double pow) {
        rightFore.setPower(pow * Constants.RIGHT_FORE_SPEED);
        rightRear.setPower(pow * Constants.RIGHT_REAR_SPEED);
    }

    private enum AngleDeltaDirection {
        POSITIVE,
        NEGATIVE
    }

    BNO055IMU gyro;

    DcMotor leftFore, leftRear, rightFore, rightRear;

    private boolean turning;
    private Gamepad prev1, prev2;
    private double targetAngle;

    protected Orientation angles;
    protected Acceleration gravity;

    private AngleDeltaDirection dir;

    @Override
    public void init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);

        leftFore = hardwareMap.dcMotor.get("lfm"); // port 2
        leftRear = hardwareMap.dcMotor.get("lrm"); // port 3
        rightFore = hardwareMap.dcMotor.get("rfm"); // port 0
        rightRear = hardwareMap.dcMotor.get("rrm"); // port 1

        leftFore.setDirection(Constants.LEFT_FORE_DIR);
        leftRear.setDirection(Constants.LEFT_REAR_DIR);
        rightFore.setDirection(Constants.RIGHT_FORE_DIR);
        rightRear.setDirection(Constants.RIGHT_REAR_DIR);

        leftFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turning = false;
    }

    @Override
    public void start() {
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, GyroAngles.ORDER, GyroAngles.UNIT);
                gravity = gyro.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return gyro.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return gyro.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return GyroAngles.formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return GyroAngles.formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return GyroAngles.formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("gravity", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    @Override
    public void loop() {
        if (turning)
            switch (dir) {
                case POSITIVE:
                    if (new GyroAngles(angles).getZ() >= targetAngle)
                        turning = false;
                    break;
                case NEGATIVE:
                    if (new GyroAngles(angles).getZ() <= targetAngle)
                        turning = false;
                    break;
            }
        else {
            setLeftPow(gamepad1.left_stick_y);
            setRightPow(gamepad1.right_stick_y);
        }

        if (gamepad1.dpad_left && !prev1.dpad_left) {
            turning = true;
            targetAngle += 90;
            setLeftPow(-0.125);
            setRightPow(0.125);
            dir = AngleDeltaDirection.POSITIVE;
        }

        if (gamepad1.dpad_right && !prev1.dpad_right) {
            turning = true;
            targetAngle -= 90;
            setLeftPow(0.125);
            setRightPow(-0.125);
            dir = AngleDeltaDirection.NEGATIVE;
        }

        telemetry.addData("Left Fore Enc.", leftFore.getCurrentPosition());
        telemetry.addData("Left Rear Enc.", leftRear.getCurrentPosition());
        telemetry.addData("Right Fore Enc.", rightFore.getCurrentPosition());
        telemetry.addData("Right Rear Enc.", rightRear.getCurrentPosition());

        telemetry.addData("Is Turning", turning);

        try {
            prev1.copy(gamepad1);
            prev2.copy(gamepad2);
        } catch (RobotCoreException e) {
            telemetry.addData("Exception", e);
        }
    }
}
