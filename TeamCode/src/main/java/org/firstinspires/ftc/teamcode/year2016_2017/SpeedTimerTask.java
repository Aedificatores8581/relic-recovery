package org.firstinspires.ftc.teamcode.year2016_2017;

import java.util.Timer;
import java.util.TimerTask;

/**
 * Created by The Saminator on 10-15-2016.
 */
public class SpeedTimerTask extends TimerTask {
    private double storedSpeed;
    private int rate;
    private PositionProvider posProv;
    private SpeedReceiver spdRecv;
    private Timer timer;

    public SpeedTimerTask(double storedSpeed, int rate, PositionProvider posProv, SpeedReceiver spdRecv, Timer timer) {
        this.storedSpeed = storedSpeed;
        this.rate = rate;
        this.posProv = posProv;
        this.spdRecv = spdRecv;
        this.timer = timer;
    }

    @Override
    public void run() {
        double pos = posProv.getPosition();
        spdRecv.setSpeed((pos - storedSpeed) / (double)(rate));
        timer.schedule(new SpeedTimerTask(pos, rate, posProv, spdRecv, timer), rate);
    }

    public interface PositionProvider {
        double getPosition();
    }

    public interface SpeedReceiver {
        void setSpeed(double speed);
    }

}