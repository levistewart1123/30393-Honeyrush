package org.firstinspires.ftc.teamcode.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {
    private MotorEx rollerMotor;
    private MotorEx conveyorMotor;
    /**
     * if our motors are drawing above their current limit, their power is multiplied by this number
     */
    private final double OVER_CURRENT_SPEED_MULTIPLIER = 0.1;
    private double rollerMult = 1;
    private double conveyorMult = 1;
    private boolean rollerOn = false;
    private boolean conveyorOn = false;
    private boolean reversed = false;

    public void initialize(HardwareMap hwMap){
        rollerMotor = new MotorEx(hwMap, "Intake rollerMotor", Motor.GoBILDA.RPM_1150);
        conveyorMotor = new MotorEx(hwMap, "Intake conveyorMotor", Motor.GoBILDA.RPM_1150);
        rollerMotor.setCachingTolerance(0.01);
        conveyorMotor.setCachingTolerance(0.01);
        rollerMotor.setCurrentAlert(100, CurrentUnit.AMPS); //TODO tune this to a reasonable value
        conveyorMotor.setCurrentAlert(100, CurrentUnit.AMPS);
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
        conveyorOn = true;
    })
            .requiring(this);

    public Command stopConveyor = instant(() -> {
        conveyorOn = false;
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

    public Command toggleReversed = instant(() -> reversed = !reversed)
            .requiring(this);

    public void update(){
        if (conveyorMotor.isOverCurrent()){
            conveyorMult = OVER_CURRENT_SPEED_MULTIPLIER;
        } else {
            conveyorMult = 1;
        }
        if (rollerMotor.isOverCurrent()){
            rollerMult = OVER_CURRENT_SPEED_MULTIPLIER;
        } else {
            rollerMult = 1;
        }
        if (reversed){
            conveyorMult *= -1;
            rollerMult *= -1;
        }

        //TODO double check if not checking if the new power is different increases loop times
        if (conveyorOn){
            conveyorMotor.set(1 * conveyorMult);
        }
        if (rollerOn){
            rollerMotor.set(1 * rollerMult);
        }
    }


}
