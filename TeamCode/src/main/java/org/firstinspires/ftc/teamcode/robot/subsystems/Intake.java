package org.firstinspires.ftc.teamcode.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * intake subsystem with separate roller and conveyor belt motors.
 */
public class Intake {
    private MotorEx rollerMotor;
//    private MotorEx transferMotor;
    /**
     * if our motors are drawing above their current limit, their power is multiplied by this number
     */
    private final double OVER_CURRENT_SPEED_MULTIPLIER = 0.1;
    private boolean rollerOn = false;
    private boolean transferOn = false;
    private boolean reversed = false;
    double rollerPower = 0, transferPower = 0;


    public void initialize(HardwareMap hwMap){
        rollerMotor = new MotorEx(hwMap, "Intake rollerMotor", Motor.GoBILDA.BARE);
        rollerMotor.setInverted(true);
//        transferMotor = new MotorEx(hwMap, "Intake transferMotor", Motor.GoBILDA.RPM_1150);
        rollerMotor.setCachingTolerance(0.01);
//        transferMotor.setCachingTolerance(0.01);
//        rollerMotor.setCurrentAlert(100, CurrentUnit.AMPS); //TODO tune this to a reasonable value
//        transferMotor.setCurrentAlert(100, CurrentUnit.AMPS);
    }

    public Command startRoller = instant(() -> {
        rollerOn = true;
    })
            .requiring(this);

    public Command stopRoller = instant(() -> {
        rollerOn = false;
    })
            .requiring(this);

    public Command startConveyor = instant(() -> {
        transferOn = true;
    })
            .requiring(this);

    public Command stopConveyor = instant(() -> {
        transferOn = false;
    })
            .requiring(this);

    public Command startAll = parallel(
            startRoller,
            startConveyor
    )
            .requiring(this);

    public Command stopAll = parallel(
            stopRoller,
            stopConveyor
    )
            .requiring(this);

    public Command reverse = instant(() -> reversed = true)
            .requiring(this);
    public Command unreverse = instant(() -> reversed = false)
            .requiring(this);

    public Command startReversed = parallel(
            reverse,
            startAll
    )
            .requiring(this);

    public Command startNormal = parallel(
            unreverse,
            startAll
    )
            .requiring(this);
    /**
     * this prevents pollen from popping out the top when the tray is lifted
     * TODO tune/remove this
     */
    public Command lowerPollen = sequential(
            unreverse,
//            instant(() -> transferMotor.setInverted(true)),
            startAll,
            waitMs(500),
            stopAll//,
//            instant(() -> transferMotor.setInverted(false))
    )
            .requiring(this)
            .setPriority(1);


    public void setPowers(double rollerPower, double transferPower){
        this.rollerPower = rollerPower;
        this.transferPower = transferPower;
    }

    /**
     * unfinished but works
     */
    public void update(){
////        if (transferMotor.isOverCurrent()){
//            conveyorMult = OVER_CURRENT_SPEED_MULTIPLIER;
//        } else {
//            conveyorMult = 1;
//        }
//        if (rollerMotor.isOverCurrent()){
//            rollerMult = OVER_CURRENT_SPEED_MULTIPLIER;
//        } else {
//            rollerMult = 1;
//        }

        rollerMotor.set(rollerPower);
//        transferMotor.set(transferPower);
    }


}
