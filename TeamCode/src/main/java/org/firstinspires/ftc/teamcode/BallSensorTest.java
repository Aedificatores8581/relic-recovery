package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name = "ballSensorTest", group = "bepis")
@Disabled
public class BallSensorTest extends OpMode {
    private VuforiaLocalizer vuforia;
    private int encoderAmount;
    private final int ONE_SECOND = 1000;
    boolean redAliance = false;

    private int cameraMonitorViewId;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    private enum ROBOT_ACTIVITY_STATE {reading, moving}
    private ROBOT_ACTIVITY_STATE state;
    DcMotor left, right, arm, motor;
    Servo grab, finger;
    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;
    byte position = 0;
    byte side = 0;
    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("lm");
        motor = hardwareMap.dcMotor.get("motor");
        right = hardwareMap.dcMotor.get("rm");
        arm = hardwareMap.dcMotor.get("am");
        grab = hardwareMap.servo.get("gr");
        finger = hardwareMap.servo.get("fr");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color sensor");
        left.setDirection(Constants.LEFT_DIR);
        right.setDirection(Constants.RIGHT_DIR);
        arm.setDirection(Constants.ARM_DIR);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);


        state = ROBOT_ACTIVITY_STATE.reading;
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdNikLv/////AAAAGWU+M9z3/00mqU+dDTTtHfZ20J0oyIXsfm2hNe0Oy/LXv4LbAaeEkgXQoLcO6ks5K0ixdWt+3WIRzmcncN31UCbuk1UJkfKtJ8IcaY+zBJe8jTlAyupXFBvONjLNShkis/kU0LHMVhFTgJZVCVaVWjaQ21nnfYHq9I2UNU1bq8+CHBDYD62VvGdSY4jwwJRgR4Rq+HYOpj/4m6P/XyqnDmFPWzF/V3If1FJaQj5E3ZZRm4lKSzvWhClfrdX/LwTkTpf3/j8QOJYEvhe9JkUwppMiKXp1iy/wEgNRFMjJKPLU5VtAqQYh/zsSEhfpeyryPGfU123eSJQoCQpq/f3Sjn37iR0ILx8dsnT1mlVStrm8";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        if(gamepad1.a)
            position = 0;
        if(gamepad1.y)
            position = 1;
        if(gamepad1.x)
            redAliance = false;
        if(gamepad1.b)
            redAliance = true;
        if(gamepad1.left_bumper)
            side = 0;
        if(gamepad1.right_bumper)
            side = 1;



    }
    public BallSensorTest() {

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
        relicTrackables.activate();

        telemetry.addData("activated", "");

/*        try{
            Thread.sleep(ONE_SECOND);
        }
        catch(InterruptedException e){
            e.printStackTrace();
        }
*/

    }
    @Override
    public void loop() {
        colors = colorSensor.getNormalizedColors();
        double redRatio = colors.red / (colors.red + colors.blue + colors.green);
        double blueRatio = colors.blue / (colors.red + colors.blue + colors.green);
        String column = "";



        grab.setPosition(0.423 + .17);


        if (redAliance == true) {
            if (redRatio >= 0.55 && redRatio > blueRatio) {
                finger.setPosition(1.0);

            } else if (blueRatio >= 0.4 && redRatio < blueRatio) {
                finger.setPosition(0.0);
            } else
                finger.setPosition(0.6044444);

        }
        if (redAliance == false) {
            if (redRatio >= 0.55 && redRatio > blueRatio) {
                finger.setPosition(0.0);
            } else if (blueRatio >= 0.4 && redRatio < blueRatio) {
                finger.setPosition(1.0);
            } else
                finger.setPosition(0.6044444);

        }





        RelicRecoveryVuMark vuMark;
        if (state == ROBOT_ACTIVITY_STATE.reading) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            switch (vuMark) {
                case LEFT:
                    //state = ROBOT_ACTIVITY_STATE.moving;
                    //encoderAmount = 8000;
                    column = "Left";
                    if(position == 0 || side == 0){
                        encoderAmount = 0;
                    }
                    if(position == 1 || side == 0){
                        encoderAmount = 0;
                    }
                    if(position == 0 || side == 1){
                        encoderAmount = 0;
                    }
                    if(position == 1 || side == 1){
                        encoderAmount = 0;
                    }
                    break;
                case CENTER:
                    //state = ROBOT_ACTIVITY_STATE.moving;
                    column = "Center";
                    if(position == 0 || side == 0){
                        encoderAmount = 0;
                    }
                    if(position == 1 || side == 0){
                        encoderAmount = 0;
                    }
                    if(position == 0 || side == 1){
                        encoderAmount = 400;
                    }
                    if(position == 1 || side == 1){
                        encoderAmount = 0;
                    }
                    //encoderAmount = 12000;
                    break;
                case RIGHT:
                    //state = ROBOT_ACTIVITY_STATE.moving;
                    column = "Right";
                    if(position == 0 || side == 0){
                        encoderAmount = 0;
                    }
                    if(position == 1 || side == 0){
                        encoderAmount = 0;
                    }
                    if(position == 0 || side == 1){
                        encoderAmount = 0;
                    }
                    if(position == 1 || side == 1){
                        encoderAmount = 0;
                    }
                    //encoderAmount = 16000;
                    break;
                default:
                    //state = ROBOT_ACTIVITY_STATE.reading;
                    break;
            }

            //671 - 683
            //get off platform\
//941-945
            //get off platform to turn
//turn
            //-355-377

//566-1279
            if(column != ""){
                if(position == 0 || side == 0) {
                    setLeftPow(0.7);
                    setRightPow(0.7);
                    //forward
                    if (left.getCurrentPosition() < encoderAmount || right.getCurrentPosition() < encoderAmount) {
                        setLeftPow(0);
                        setRightPow(0);
                        //left
                        if(left.getCurrentPosition() > 0 || right.getCurrentPosition() > 0) {
                            setLeftPow(0);
                            setRightPow(0);
                            motor.setPower(0.7);
                            if(motor.getCurrentPosition() > 0) {
                                setLeftPow(0);
                                setRightPow(0);
                                motor.setPower(0);
                                //right
                            }

                        }

                    }
                }
                if(position == 0 || side == 1){
                    setLeftPow(0);
                    setRightPow(0);
                    //backwards
                    if(left.getCurrentPosition() < 908 || right.getCurrentPosition() < 888) {
                        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        setLeftPow(-0.8);
                        setRightPow(0.8);
                        //left

                            if(left.getCurrentPosition() < -321 || right.getCurrentPosition() > 342){
                                left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                left.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
                                setLeftPow(0.6);
                                setRightPow(0.6);
                                if(left.getCurrentPosition() > encoderAmount || right.getCurrentPosition() > encoderAmount) {
                                    while(left.getCurrentPosition() == encoderAmount - 350 || right.getCurrentPosition() == encoderAmount + 350)
                                    left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                    right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                    right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                    left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                    motor.setPower(0.7);
                                    if (motor.getCurrentPosition() > 50) {
                                        setLeftPow(0);
                                        setRightPow(0);
                                        motor.setPower(0);
                                        //right

                                    }
                                }
                        }
                        }
                }
                if(position == 1 || side == 0){

                        setLeftPow(0.7);
                        setRightPow(0.7);
                    //forward
                        if(left.getCurrentPosition() > 0 || right.getCurrentPosition() > 0) {
                            setLeftPow(0);
                            setRightPow(0);
                            //right
                            if (left.getCurrentPosition() < encoderAmount || right.getCurrentPosition() < encoderAmount) {
                                setLeftPow(0);
                                setRightPow(0);
                                //forward
                                if(left.getCurrentPosition() > 0 || right.getCurrentPosition() > 0) {
                                    setLeftPow(0);
                                    setRightPow(0);
                                    //left
                                    if(left.getCurrentPosition() > 0 || right.getCurrentPosition() > 0) {
                                        setLeftPow(0);
                                        setRightPow(0);
                                        motor.setPower(0.7);
                                        if(motor.getCurrentPosition() > 0) {
                                            setLeftPow(0);
                                            setRightPow(0);
                                            motor.setPower(0);
                                            //right
                                        }

                                    }
                                }
                            }
                        }


                    //}
                }
                if(position == 1 || side == 1){
                    if(left.getCurrentPosition() > 0 || right.getCurrentPosition() > 0) {
                        setLeftPow(0);
                        setRightPow(0);
                        //backwards
                        if(left.getCurrentPosition() > 0 || right.getCurrentPosition() > 0) {
                            setLeftPow(0);
                            setRightPow(0);
                            //right
                            if(left.getCurrentPosition() < encoderAmount || right.getCurrentPosition() < encoderAmount) {
                                setLeftPow(0);
                                setRightPow(0);
                                //forward
                                if(left.getCurrentPosition() > 0 || right.getCurrentPosition() > 0) {
                                    setLeftPow(0);
                                    setRightPow(0);
                                    //right
                                    if(left.getCurrentPosition() > 0 || right.getCurrentPosition() > 0) {
                                        setLeftPow(0);
                                        setRightPow(0);
                                        motor.setPower(0.7);
                                        if(motor.getCurrentPosition() > 0) {
                                            setLeftPow(0);
                                            setRightPow(0);
                                            motor.setPower(0);
                                            //right
                                        }

                                    }
                                }
                            }
                        }
                    }
                }

            }


            telemetry.addLine()
                    .addData("a", colors.alpha)
                    .addData("red Ratio", (colors.red / (colors.blue + colors.red + colors.green)))
                    .addData("green Ratio", (colors.green / (colors.blue + colors.red + colors.green)))
                    .addData("blue Ratio", (colors.blue / (colors.blue + colors.red + colors.green)))
                    .addData("blue", colors.blue)
                    .addData("red", colors.red)
                    .addData("column", column)
                    .addData("finger", finger.getPosition());
            telemetry.update();
        }
    }


    public void stop() {
        setLeftPow(0.0);
        setRightPow(0.0);

    }

    protected void setLeftPow(double pow) {
        left.setPower(pow * Constants.LEFT_SPEED);
    }
    protected void setRightPow(double pow) {
        right.setPower(pow * Constants.RIGHT_SPEED);
    }

    protected boolean checkEncoder(int ticks) {
        int distance = Math.abs(ticks);
        int leftDist = Math.abs(left.getCurrentPosition());
        int rightDist = Math.abs(right.getCurrentPosition());

        return (distance <= leftDist) || (distance <= rightDist);
    }
}