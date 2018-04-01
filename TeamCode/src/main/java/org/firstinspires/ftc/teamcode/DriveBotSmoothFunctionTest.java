package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Hunter Seachrist on 3/6/2018.
 */

@Disabled
@Autonomous(name = "Smooth Drive Function Test", group = "Bepis")
public class DriveBotSmoothFunctionTest extends DriveBotTestTemplate{
    private enum SmoothieState { // I like Fruit smoothies
        FORWARD_MOVE(1000, 0.2, 0.4), RESET_ENCODERS(0,0,0), BACKWARD_MOVE(-1000, 0.2, 0.4), STOP(0,0,0);

        protected int totalEncoders;
        protected double percentTimeChangingSpeedPerState;
        protected double maxPower;

        SmoothieState(int totalEncoders, double percentTimeChangingSpeedPerState, double maxPower){
            this.totalEncoders = totalEncoders;
            this.percentTimeChangingSpeedPerState = percentTimeChangingSpeedPerState;
            this.maxPower = maxPower;
        }
    }

    SmoothieState smoothieState;
    double leftPow;
    double rightPow;

    public void start(){
        super.start();
        smoothieState = SmoothieState.FORWARD_MOVE;
        leftPow = 0.0;
        rightPow = 0.0;
        resetEncoders();

        leftFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void loop(){



        switch (smoothieState){
            case FORWARD_MOVE:
                if ((leftFore.getCurrentPosition() + rightFore.getCurrentPosition()) / 2 >= 0.999 * SmoothieState.FORWARD_MOVE.totalEncoders){
                    smoothieState = SmoothieState.RESET_ENCODERS;
                }

                leftPow = (getPowerSmootheDrive(leftFore.getCurrentPosition(),
                        SmoothieState.FORWARD_MOVE.totalEncoders,
                        SmoothieState.FORWARD_MOVE.percentTimeChangingSpeedPerState,
                        SmoothieState.FORWARD_MOVE.maxPower));

                rightPow = (getPowerSmootheDrive(rightFore.getCurrentPosition(),
                        SmoothieState.FORWARD_MOVE.totalEncoders,
                        SmoothieState.FORWARD_MOVE.percentTimeChangingSpeedPerState,
                        SmoothieState.FORWARD_MOVE.maxPower));
                break;

            case RESET_ENCODERS:
                leftPow = 0.0;
                rightPow = 0.0;
                resetEncoders();
                leftFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                smoothieState = SmoothieState.BACKWARD_MOVE;
                break;

            case BACKWARD_MOVE:
                if (Math.abs(leftFore.getCurrentPosition() + rightFore.getCurrentPosition()) / 2 >= Math.abs(0.999 * SmoothieState.BACKWARD_MOVE.totalEncoders)){
                    smoothieState = SmoothieState.STOP;
                }

                leftPow = (getPowerSmootheDrive(leftFore.getCurrentPosition(),
                        SmoothieState.BACKWARD_MOVE.totalEncoders,
                        SmoothieState.BACKWARD_MOVE.percentTimeChangingSpeedPerState,
                        SmoothieState.BACKWARD_MOVE.maxPower));

                rightPow = (getPowerSmootheDrive(rightFore.getCurrentPosition(),
                        SmoothieState.BACKWARD_MOVE.totalEncoders,
                        SmoothieState.BACKWARD_MOVE.percentTimeChangingSpeedPerState,
                        SmoothieState.BACKWARD_MOVE.maxPower));
                break;

            case STOP:
                leftPow = 0;
                rightPow = 0;
                break;
        }

        setLeftPow(leftPow);
        setRightPow(rightPow);

        telemetry.addData("State", smoothieState);
        telemetry.addData("Left Power", leftPow);
        telemetry.addData("Left Enc", leftFore.getCurrentPosition());

        telemetry.addData("\nRight Power", rightPow);
        telemetry.addData("Right Enc", rightFore.getCurrentPosition());

    }
}
