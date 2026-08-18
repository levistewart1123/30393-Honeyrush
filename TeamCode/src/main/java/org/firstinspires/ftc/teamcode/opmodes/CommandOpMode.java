package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.Timing;

import java.util.concurrent.TimeUnit;

/**
 * This is the base class that all OpModes we make should extend.
 * It automatically logs loop times and runs the scheduler.
 * @author Levi
 */
public class CommandOpMode extends OpMode {
    protected double loops = 0;
    protected double secondLoops = 0;
    protected double storedLoops = 0;
    protected ElapsedTime loopTimer = new ElapsedTime();
    protected Timing.Timer secTimer = new Timing.Timer(1000, TimeUnit.MILLISECONDS);
    public void reset() {
        Scheduler.reset();
    }

    /**
     * Schedules commands to the scheduler
     */
    public void schedule(Command... commands) {
        Scheduler.schedule(commands);
    }

    /**
     * resets scheduler
     */
    @Override
    public void init() {
        reset();
    }

    /**
     * starts timers for loop time logging and runs loop once
     */
    @Override
    public void start() {
        loopTimer.reset();
        secTimer.start();
        loop();
    }

    /**
     * adds loop time to telemetry, updates it, and runs scheduler
     */
    @Override
    public void loop() {
        secondLoops++;
        loops++;
        if (secTimer.done()){
            storedLoops = secondLoops;
            secondLoops = 0;
            secTimer.start();
        }
        telemetry.addData("average loop time (ms): ", (loopTimer.milliseconds()/loops));
        telemetry.addData("loops per second (approximate)", storedLoops);
        telemetry.update();
        Scheduler.execute();
    }

    /**
     * resets scheduler
     */
    public void stop() {
        reset();
    }
}
