package org.firstinspires.ftc.teamcode.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Tray {
    private MotorGroup slides;
    private ServoEx door;

    private final int upPosition = 1000; //in encoder ticks TODO tune this
    private final int downPosition = 0;

    public boolean isUp = false;
    public boolean isDown = true;
    private boolean targetIsUpPosition = false;

    public void initialize(HardwareMap hwMap){
        MotorEx leftSlide = new MotorEx(hwMap, "Tray leftSlide");//TODO add gobilda type/(cpr and rpm)
        MotorEx rightSlide = new MotorEx(hwMap, "Tray rightSlide");
        slides = new MotorGroup(leftSlide, rightSlide);
        slides.setRunMode(Motor.RunMode.PositionControl);
        slides.setPositionCoefficient(0.05); //TODO tune this (kP)
        slides.setPositionTolerance(2); //TODO tune this
        slides.stopAndResetEncoder();

        door = new ServoEx(hwMap, "Tray door"); //TODO maybe add range
        door.setCachingTolerance(0.02);
    }

    public Command setUp = instant(() -> {
        slides.setTargetPosition(upPosition);
        targetIsUpPosition = true;
    });

    public Command setDown = instant(() -> {
        slides.setTargetPosition(downPosition);
        targetIsUpPosition = false;
    });
    public void openDoor(){
        door.set(1);
    }
    public void closeDoor(){
        door.set(0);
    }

    public Command open = instant(this::openDoor);
    public Command close = instant(this::closeDoor);



    public void update(){
        if (!slides.atTargetPosition()) {
            slides.set(0.75); //todo tune this
        } else {
            slides.stopMotor();
        }
        if (slides.atTargetPosition()){
            if (targetIsUpPosition) {
                isUp = true;
                isDown = false;
            } else {
                isDown = true;
                isUp = false;
            }
        } else {
            isUp = false;
            isDown = false;
        }
    }
}
