package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * Created by The Saminator on 06-29-2017.
 */
@Autonomous(name = "ballsensor", group = "bepis")
public abstract class SmolBotArm extends OpMode {
    DcMotor left, right, arm, hand;
    Servo grab;
    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;

    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("lm");
        right = hardwareMap.dcMotor.get("rm");
        arm = hardwareMap.dcMotor.get("am");
        hand = hardwareMap.dcMotor.get("hm");
        grab = hardwareMap.servo.get("gr");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color sensor");
        left.setDirection(Constants.LEFT_DIR);
        right.setDirection(Constants.RIGHT_DIR);
        arm.setDirection(Constants.ARM_DIR);
        hand.setDirection(Constants.HAND_DIR);
        grab.setDirection(Constants.GRAB);


    }
    public SmolBotArm() {

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
    }
    @Override
    public void loop(){
        colors = colorSensor.getNormalizedColors();
        double blue = colors.blue;
        double red = colors.red;
        double redRatio = colors.red / (colors.red + colors.blue + colors.green);
        double blueRatio = colors.blue / (colors.red + colors.blue + colors.green);
        boolean blueAliance = true;


/*        right.setPower(1);
        left.setPower(1);
        if(blueAliance == true)
            if (red == 0 && redRatio * 1 > blueRatio)
                stop();
                setGrabPow (1);
            if (blue == 0 && redRatio * 1 < blueRatio)
                stop();
                setGrabPow (-1);
        if(blueAliance == false)
            if (red == 0 && redRatio * 1 > blueRatio)
                stop();
                setGrabPow (-1);
            if (blue == 0 && redRatio * 1 < blueRatio)
                stop();
                setGrabPow (1);
                */
        telemetry.addLine()
                .addData("a", colors.alpha )
                .addData("red Ratio", (colors.red/(colors.blue + colors.red + colors.green)))
                .addData("green Ratio", (colors.green/(colors.blue + colors.red + colors.green)))
                .addData("blue Ratio", (colors.blue/(colors.blue + colors.red + colors.green)))
                .addData("blue", colors.blue)
                .addData("red", colors.red);
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

    protected void setArmPow(double pow) {
        arm.setPower(pow * Constants.ARM_SPEED);
    }


    protected void setGrabPow(double position) {
        grab.setPosition(position * Constants.GRAB_SPEED);
    }

    protected void setHandPow(double pow) {
        hand.setPower(pow * Constants.HAND_SPEED);
    }

    protected boolean checkEncoder(int ticks) {
        int distance = Math.abs(ticks);
        int leftDist = Math.abs(left.getCurrentPosition());
        int rightDist = Math.abs(right.getCurrentPosition());

        return (distance <= leftDist) || (distance <= rightDist);
    }
}
