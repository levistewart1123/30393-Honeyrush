package org.firstinspires.ftc.teamcode.robot.subsystems;

import static java.lang.Math.max;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

/**
 * Uses butterfly wheels and a PTO (not implemented).
 * This only does tank drive and lifting/lowering the wheels.
 * For mecanum and any path following, use pedropathing and make sure the wheels are up.
 *
 */
public class Drivetrain {
    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private ServoEx leftServo, rightServo;
    private final double LEFT_UP_POS = 0, LEFT_DOWN_POS = 1, RIGHT_UP_POS = 0, RIGHT_DOWN_POS = 1;
    public boolean wheelsUp = true;


    public void initialize(HardwareMap hwMap){
        frontLeft = new MotorEx(hwMap, "Drivetrain frontLeft");
        frontRight = new MotorEx(hwMap, "Drivetrain frontRight");
        backLeft = new MotorEx(hwMap, "Drivetrain backLeft");
        backRight = new MotorEx(hwMap, "Drivetrain backRight");

        MotorEx[] motors = {frontLeft, frontRight, backLeft, backRight};

        for (MotorEx motor: motors) {
            motor.setCachingTolerance(0.005);
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motor.setRunMode(Motor.RunMode.RawPower);
        }
        backLeft.setInverted(true);
        backRight.setInverted(true);

        leftServo = new ServoEx(hwMap, "ButterflyDrivetrain leftServo")
                .setCachingTolerance(0.01);
        rightServo = new ServoEx(hwMap, "ButterflyDrivetrain rightServo")
                .setCachingTolerance(0.01);

    }

    public void drive(double forward, double turn){
        double left = forward + turn;
        double right = forward - turn;
        double largest = max(left, right);
        if (largest > 1){
            left /= largest;
            right /= largest;
        }

        frontLeft.set(left);
        backLeft.set(left);
        frontRight.set(right);
        backRight.set(right);
    }

    public void liftButterflyWheels(){
        leftServo.set(LEFT_UP_POS);
        rightServo.set(RIGHT_UP_POS);
        wheelsUp = true;
    }
    public void lowerButterflyWheels(){
        leftServo.set(LEFT_DOWN_POS);
        rightServo.set(RIGHT_DOWN_POS);
        wheelsUp = false;
    }
}
