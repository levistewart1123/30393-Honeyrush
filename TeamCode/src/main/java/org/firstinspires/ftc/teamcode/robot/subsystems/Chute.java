package org.firstinspires.ftc.teamcode.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Chute {
    private ServoEx gate; //done use solverslib ServoEx
    private MotorEx liftL; //use solverslib MotorEx and/or MotorGroup
    private MotorEx liftR;
    private RevTouchSensor limitSwtich; //I didn't realize that we had a limit switch, where is it?

    int targetPosition = 0; 

    public void initialize(HardwareMap hwMap){ //looks good but will have to change w/solverslib hw classes
        gate = hwMap.get(ServoEx.class,"gate");
        liftL = hwMap.get(MotorEx.class,"liftL");
        liftR = hwMap.get(MotorEx.class,"liftR");
        limitSwtich = hwMap.get(RevTouchSensor.class,"limitSwitch");

        liftL.setInverted(false);
        liftR.setInverted(true);

        //liftL.setMode(MotorEx.RunMode.RUN_TO_POSITION);
        //liftR.setMode(MotorEx.RunMode.RUN_TO_POSITION);
        liftL.setRunMode(Motor.RunMode.PositionControl);
        liftR.setRunMode(Motor.RunMode.PositionControl);

        liftL.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        liftR.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

    }

    public void open(){
        final double gateOpenPos = 0; //done make this final and move it to the top or remove the variable, change to gateOpenPosPos for clarity
        gate.set(gateOpenPos);
    }

    public void close(){
        double gateClosedPos = 0.5; //done make this final and move it to the top or remove the variable, change
        gate.set(gateClosedPos);
    }

    public void honeyCombPos(){ //change name to be clearer, start it with 
        targetPosition = 350;
    }

    public Command home = sequential(
            instant(() -> {
                if (!limitSwtich.isPressed()) {
                    liftL.setRunMode(Motor.RunMode.RawPower); //changing runmodes mid-opmode seems a little weird to me but idk
                    liftR.setRunMode(Motor.RunMode.RawPower);

                    //liftL.setPower(0.5);
                    //liftR.setPower(0.5);
                    liftL.set(-0.5);
                    liftR.set(-0.5);
                }else{
                    liftL.set(0);//this will immediately get overwritten as all of this code will run
                    liftR.set(0);
                    //liftL.setMode(MotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    //liftR.setMode(MotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    liftL.stopAndResetEncoder();
                    liftR.stopAndResetEncoder();
                    waitMs(0.03);

                    liftL.setRunMode(Motor.RunMode.PositionControl);
                    liftR.setRunMode(Motor.RunMode.PositionControl);

                    targetPosition = 0;
                    liftL.setTargetPosition(targetPosition);
                    liftR.setTargetPosition(targetPosition);

                    liftL.set(0.7);
                    liftR.set(0.7);
                }
            })
    );

    public void update(){ //done should be called update() and not capitalized
        liftL.setTargetPosition(targetPosition);
        liftR.setTargetPosition(targetPosition);
    }

}
