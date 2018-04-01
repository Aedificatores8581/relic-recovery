package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@TeleOp(name = "ballSensorTele", group = "bepis")
@Disabled
public class BallSensorTele extends OpMode {
    DcMotor left, right, motor;
    Servo grab, finger, s1, s2;
    CRServo crs1, crs2;
    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;
    double s1position = 0.75;
    double INCREMENT = 0.005;     // amount to slew servo each CYCLE_MS cyclE
    double INCREMENT2 = .02;
    double MAX_POS = .75;     // Maximum rotational position
    double MIN_POS = .25;     // Minimum rotational position
    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("lm");
        right = hardwareMap.dcMotor.get("rm");
        grab = hardwareMap.servo.get("gr");
        finger = hardwareMap.servo.get("fr");

        motor = hardwareMap.dcMotor.get("motor");
        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");
        crs2 = hardwareMap.crservo.get("crs2");
        crs1 = hardwareMap.crservo.get("crs1");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color sensor");
        left.setDirection(Constants.LEFT_DIR);
        right.setDirection(Constants.RIGHT_DIR);
        grab.setDirection(Servo.Direction.REVERSE);


//        finger.scaleRange(0.0, 1.0);


    }

    public BallSensorTele() {

        // Initialize base classes.
        //
        // All via self-construction.
        //
        // Initialize class members.
        //
        // All via self-construction.


    }

    @Override
    public void start() {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);





    }
    double servoPosition = .5;
    double servoPosition1 = 0.17;
    double max = 1.0;
    double min = 0.0;
    @Override


    public void loop(){
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double Left = gamepad1.left_stick_y;
        double Right = gamepad1.right_stick_y;

        left.setPower(Left * 0.6);
        right.setPower(Right * 0.6);
        if(gamepad1.left_bumper)
            System.out.println("left motor: " + left.getCurrentPosition() + "   right motor: " + right.getCurrentPosition());

        if (gamepad1.a){
            servoPosition += .001;
            if (servoPosition >= max)
                servoPosition = max;

            finger.setPosition(servoPosition);
        }
        if(gamepad1.right_stick_button && gamepad1.left_stick_button){
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
//671 - 683
        //get off platform\
//941-945
        //get off platform to turn
//turn
        //-355-377

//566-1279
        else if (gamepad1.b){
            servoPosition -= .001;
            if (servoPosition <= min) {
                servoPosition = min;
            }
            finger.setPosition(servoPosition);
        }
        else if(gamepad1.x){
            servoPosition1 += .001;
            if (servoPosition1 >= max) {
                servoPosition1 = max;
            }
            grab.setPosition(servoPosition1);
        }
        else if(gamepad1.y){
            servoPosition1 -= .001;
            if (servoPosition1 <= min) {
                servoPosition1 = min;
            }
            grab.setPosition(servoPosition1);
        }


        telemetry.addLine()
//                .addData("finger", finger.getPosition())
//                .addData("servoposition", servoPosition)
//               .addData("servoposition1", servoPosition1)
                .addData("left", left.getCurrentPosition())
                .addData("right", right.getCurrentPosition());
        telemetry.update();
        if (gamepad2.dpad_up)
            motor.setPower(-0.5);
        if (gamepad2.dpad_down)
            motor.setPower(0.5);
        if (!(gamepad2.dpad_up ^ gamepad2.dpad_down))
            motor.setPower(0);
        if (gamepad2.left_stick_y < 0) {
            // Keep stepping up until we hit the max value.
            s1position += INCREMENT;
            if (s1position >= MAX_POS) {
                s1position = MAX_POS;
            }
            s1.setPosition(s1position);
        }
        if (gamepad2.left_stick_y > 0) {
            s1position -= INCREMENT;
            if (s1position <= MIN_POS) {
                s1position = MIN_POS;
            }
            s1.setPosition(s1position);
        }
        if (gamepad2.right_stick_y == 0) {
            crs1.setPower(0);
        }
        if (gamepad2.right_stick_y < 0) {
            crs1.setDirection(DcMotorSimple.Direction.REVERSE);
            crs1.setPower(1);
        }
        if (gamepad2.right_stick_y > 0) {
            crs1.setDirection(DcMotorSimple.Direction.FORWARD);
            crs1.setPower(1);
        }
        if (gamepad2.x) {
            crs2.setPower(0);
        }
        if (gamepad2.y) {
            crs2.setDirection(DcMotorSimple.Direction.FORWARD);
            crs2.setPower(0.2);
        }
        if (gamepad2.a) {
            crs2.setDirection(DcMotorSimple.Direction.REVERSE);
            crs2.setPower(0.2);
        }
        if(gamepad2.b){
            crs2.setDirection(DcMotorSimple.Direction.FORWARD);
            crs2.setPower(.01);

        }


        telemetry.addData("Status", "Running");
        telemetry.addData("Servo Position 1", "%5.2f", s1.getPosition());
        telemetry.addData("Servo Position 2", "%5.2f", s2.getPosition());
        telemetry.update();



    }

    }

