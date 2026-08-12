package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PoseSaver;
import org.firstinspires.ftc.teamcode.robot.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Tray;


import java.util.List;

/**
 * This holds all of our subsystem classes and puts together Commands using them.
 */
@Configurable
public class Robot {
    public boolean isRed;
    public boolean slowDrive = false;
    public Follower follower;
    public Drivetrain drivetrain = new Drivetrain();
    public Tray tray = new Tray();
    public Intake intake = new Intake();
    private final double SLOW_MODE_MULTIPLIER = 0.2;
    public boolean butterflyWheelsDown = false;


    public void initialize(boolean isRed, HardwareMap hwMap) {
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); //TODO try setting this to manual and see how much loop times improve
        }

        follower = Constants.createFollower(hwMap);
        intake.initialize(hwMap);
        tray.initialize(hwMap);

        this.isRed = isRed;
        if (PoseSaver.autoWasRun) {
            follower.setStartingPose(PoseSaver.endPose);
        } else {
            follower.setStartingPose(new Pose(10, 10, 0));
        }
        PoseSaver.autoWasRun = false;
        follower.update();
    }

    public void update() {
        if (follower.isBusy()) drivetrain.liftButterflyWheels();
        follower.update();
        tray.update();
        intake.update();
    }

    public Command dump = parallel(
            sequential(
                    instant(() -> slowDrive = true),
                    tray.setUp,
                    waitUntil(() -> tray.isUp),
                    tray.open,
                    waitMs(2000),//TODO figure out how long it takes balls to exit
                    tray.close,
                    instant(() -> slowDrive = false),
                    tray.setDown,
                    waitUntil(() -> tray.isDown)
            ),
            intake.lowerPollen
    )
            .requiring(intake, tray)
            .setPriority(2);

    public void drive(double forward, double strafe, double turn){
        if (slowDrive) {
            forward *= SLOW_MODE_MULTIPLIER;
            strafe *= SLOW_MODE_MULTIPLIER;
            turn *= SLOW_MODE_MULTIPLIER;
        }
        if (drivetrain.wheelsUp) {
            if (!follower.isTeleopDrive()) follower.startTeleOpDrive();
            follower.setTeleOpDrive(forward, strafe, turn);
        } else {
            drivetrain.drive(forward, turn);
        }

    }




}
