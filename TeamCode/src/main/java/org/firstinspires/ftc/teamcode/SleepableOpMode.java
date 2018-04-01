package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Created by The Saminator on 11-18-2017.
 */
public abstract class SleepableOpMode extends OpMode {
    public abstract class SleepableTask implements Comparable<SleepableTask> {
        private long timeGoal;

        public SleepableTask(int millisToWait) {
            timeGoal = System.currentTimeMillis() + millisToWait;
        }

        public long getTimeGoal() {
            return timeGoal;
        }

        public int compareTo(SleepableTask other) {
            return Long.compare(this.timeGoal, other.timeGoal);
        }

        public abstract void run();
    }

    protected List<SleepableTask> tasks = new ArrayList<>();

    public void addTask(SleepableTask task) {
        tasks.add(task);
        Collections.sort(tasks);
    }

    @Override
    public void loop() {
        if (tasks.size() > 0)
            for (;;) {
                SleepableTask task = tasks.get(0);
                if (System.currentTimeMillis() <= task.getTimeGoal())
                    break;
                task.run();
                tasks.remove(0);
            }
    }
}
