package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.CommandOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;

@Configurable
@TeleOp
public class DtTuner extends CommandOpMode {
    private Drivetrain drivetrain = new Drivetrain();

    @Override
    public void init() {
        super.init();
        drivetrain.initialize(hardwareMap);
    }

    @Override
    public void start() {
        super.start();
        drivetrain.liftButterflyWheels();
    }

    @Override
    public void loop() {
        drivetrain.update();
        if (!drivetrain.ptoEnabled) {
            drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x); //may need to invert
        }

        if (gamepad1.yWasPressed()) drivetrain.slowDrive = true;

        if (gamepad1.left_bumper) {
            drivetrain.liftButterflyWheels();
        } else if (gamepad1.right_bumper) {
            drivetrain.lowerButterflyWheels();
        } /*else {
            drivetrain.autoLift();
        }*/

        if (gamepad1.rightStickButtonWasPressed()) drivetrain.runPto.schedule();

        super.loop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
