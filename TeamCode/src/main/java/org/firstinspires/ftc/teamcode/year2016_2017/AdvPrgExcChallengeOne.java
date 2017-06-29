package org.firstinspires.ftc.teamcode.year2016_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Timer;
import java.util.TimerTask;

/**
 * Created by The Saminator on 09-10-2016.
 */
@Autonomous(name = "Challenger One", group = "Advanced Programming Exercises")
@Disabled
public class AdvPrgExcChallengeOne extends OpMode {

    private Robot bot;

    private boolean notStarted;

    public enum State {
        RotateLeft {
            @Override
            public void runState(final Robot bot) {
                bot.getLeft().setPower(-1.0);
                bot.getRight().setPower(1.0);
                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        bot.setState(MoveToAft);
                    }
                }, 1000);
            }
        },
        RotateRight {
            @Override
            public void runState(final Robot bot) {
                bot.getLeft().setPower(1.0);
                bot.getRight().setPower(-1.0);
                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        bot.setState(MoveToFore);
                    }
                }, 1000);
            }
        },
        MoveToFore {
            @Override
            public void runState(final Robot bot) {
                bot.getLeft().setPower(1.0);
                bot.getRight().setPower(1.0);
                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        bot.setState(RotateLeft);
                    }
                }, 1000);
            }
        },
        MoveToAft {
            @Override
            public void runState(final Robot bot) {
                bot.getLeft().setPower(-1.0);
                bot.getRight().setPower(-1.0);
                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        bot.setState(RotateRight);
                    }
                }, 1000);
            }
        },
        Starting {
            @Override
            public void runState(final Robot bot) {
                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        bot.setState(MoveToFore);
                    }
                }, 500);
            }
        },
        Stopped {
            @Override
            public void runState(Robot bot) {
                timer.cancel();
            }
        };

        private static Timer timer = new Timer();

        public abstract void runState(final Robot bot);
    }

    public static class Robot {

        private State state;
        private DcMotor left, right;
        private DcMotorController ctrl;

        public Robot(HardwareMap map) {
            left = map.dcMotor.get("lm");
            right = map.dcMotor.get("rm");
            ctrl = map.dcMotorController.get("mc1");

            left.setDirection(DcMotor.Direction.FORWARD);
            right.setDirection(DcMotor.Direction.REVERSE);
        }

        public void setState(State state) {
            this.state = state;
        }

        public State getState() {
            return this.state;
        }

        public void runState() {
            this.state.runState(this);
        }

        public DcMotor getLeft() {
            return this.left;
        }

        public DcMotor getRight() {
            return this.right;
        }
    }

    @Override
    public void init() {
        bot = new Robot(hardwareMap);
        notStarted = true;
    }

    @Override
    public void loop() {
        if (notStarted) {
            bot.setState(State.Starting);
            bot.runState();
            notStarted = false;
        }
    }

    @Override
    public void stop() {
        bot.setState(State.Stopped);
        bot.runState();
    }
}
