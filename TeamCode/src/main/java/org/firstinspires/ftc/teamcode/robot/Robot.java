package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PoseSaver;
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
    public Drivetrain drivetrain = new Drivetrain();
    public Tray tray = new Tray();
    public Intake intake = new Intake();



    public void initialize(boolean isRed, HardwareMap hwMap) {
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); //TODO try setting this to manual and see how much loop times improve
        }

        intake.initialize(hwMap);
        tray.initialize(hwMap);
        drivetrain.initialize(hwMap);

        this.isRed = isRed;
        if (PoseSaver.autoWasRun) {
            drivetrain.setStartingPose(PoseSaver.endPose);
        } else {
            drivetrain.setStartingPose(new Pose(10, 10, 0));
        }
        PoseSaver.autoWasRun = false;
    }

    /**
     * updates everything and lifts butterfly wheels if a path is being followed (will likely be changed)
     */
    public void update() {
        drivetrain.update();
        tray.update();
        intake.update();
    }

    public Command dump = parallel(
            sequential(
                    instant(() -> drivetrain.slowDrive = true),
                    tray.setUp,
                    waitUntil(() -> tray.isUp),
                    tray.open,
                    waitMs(2000),//TODO figure out how long it takes balls to exit
                    tray.close,
                    instant(() -> drivetrain.slowDrive = false),
                    tray.setDown,
                    waitUntil(() -> tray.isDown)
            ),
            intake.lowerPollen
    )
            .requiring(intake, tray)
            .setPriority(2);


}
