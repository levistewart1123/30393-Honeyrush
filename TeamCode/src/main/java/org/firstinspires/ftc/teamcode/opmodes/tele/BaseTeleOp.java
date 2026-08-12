package org.firstinspires.ftc.teamcode.opmodes.tele;


import org.firstinspires.ftc.teamcode.opmodes.CommandOpMode;
import org.firstinspires.ftc.teamcode.PoseSaver;
import org.firstinspires.ftc.teamcode.robot.Robot;


/**
 *This is our base TeleOp class.
 * Red and Blue TeleOps that extend this should be created and put on the driver station.
 */
public class BaseTeleOp extends CommandOpMode {
    protected Robot robot = new Robot();

    protected boolean isRed;
    public BaseTeleOp(boolean isRed) {
        this.isRed = isRed;
    }

    @Override
    public void init() {
        super.init();
        robot.initialize(isRed, hardwareMap);
    }

    @Override
    public void start() {
        robot.update();
        super.start();
    }

    @Override
    public void loop() {
        robot.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x); //may need to invert

        if (gamepad1.aWasPressed()) robot.dump.schedule();

        if (gamepad1.right_trigger > 0.1){
            robot.intake.startAll.schedule();
        } else {
            robot.intake.stopAll.schedule();
        }

        if (gamepad1.yWasPressed()) robot.slowDrive = true;

        robot.update();
        super.loop();
    }

    public void stop(){
        PoseSaver.autoWasRun = false;
        super.stop();
    }

}
