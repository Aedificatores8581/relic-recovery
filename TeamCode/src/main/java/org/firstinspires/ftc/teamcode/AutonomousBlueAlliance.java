package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Autonomous Relic Recovery: Blue Aliance", group = "bepis")
@Disabled
public abstract class AutonomousBlueAlliance extends OpMode {
    DcMotorController motorController;
    DcMotor forwardLeftMotor, forwardRightMotor, backwardLeftMotor, backwardRightMotor;
    NormalizedColorSensor colorSensor;
    Servo finger;
    BNO055IMU imu;
    Orientation angle;
    Acceleration gravity;

    @Override
    public void init(){
        forwardRightMotor = hardwareMap.dcMotor.get("fr");
        forwardLeftMotor = hardwareMap.dcMotor.get("fl");
        backwardLeftMotor = hardwareMap.dcMotor.get("bl");
        backwardRightMotor = hardwareMap.dcMotor.get("br");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "cs");


        //configuration goes here
    }

    public enum State {
        STATE_SCAN_PICTURE,
        STATE_EXTEND_ARM,
        STATE_READ_COLOR_SENSOR,
        STATE_KNOCK_OFF_JEWEL,
        STATE_DRIVE_TO_SHELF,
        STATE_ORIENT_WITH_SHELF,
        STATE_POSITION_LIFT,
        STATE_INSERT_BLOCK,
        STATE_CHECK_TIME,
        STATE_DRIVE_TO_CENTER,
        STATE_GRAB_BLOCK,
        STATE_DRIVE_BACK,
        STATE_END,
    }

    @Override
    public void loop() {

    }

}
