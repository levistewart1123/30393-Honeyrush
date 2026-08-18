package org.firstinspires.ftc.teamcode.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static java.lang.Math.max;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.Timing;

import java.util.concurrent.TimeUnit;

/**
 * Uses butterfly wheels and a PTO (not implemented).
 * This only does tank drive and lifting/lowering the wheels.
 * For mecanum and any path following, use pedropathing and make sure the wheels are up.
 *
 */
public class Drivetrain {
    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private ServoEx leftButterflyServo, rightButterflyServo, leftPto, rightPto;
    private TouchSensor leftTouch, rightTouch, backTouch, frontTouch;
    private final double LEFT_WHEEL_UP_POS = 0,
            LEFT_WHEEL_DOWN_POS = 1,
            RIGHT_WHEEL_UP_POS = 0,
            RIGHT_WHEEL_DOWN_POS = 1,
            RIGHT_PTO_OUT_POS = 1,
            RIGHT_PTO_IN_POS = 0,
            LEFT_PTO_OUT_POS = 1,
            LEFT_PTO_IN_POS = 0;

    public boolean wheelsUp = true;
    public boolean ptoEnabled = false;
    private Timing.Timer lowerWhenUntouchedTimer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
    private boolean wasPressed = false;


    public void initialize(HardwareMap hwMap){
        frontLeft = new MotorEx(hwMap, "Drivetrain frontLeft").setCachingTolerance(0.005);
        frontRight = new MotorEx(hwMap, "Drivetrain frontRight").setCachingTolerance(0.005);
        backLeft = new MotorEx(hwMap, "Drivetrain backLeft").setCachingTolerance(0.005);
        backRight = new MotorEx(hwMap, "Drivetrain backRight").setCachingTolerance(0.005);

        backLeft.setInverted(true);
        backRight.setInverted(true);

        leftButterflyServo = new ServoEx(hwMap, "Drivetrain leftButterflyServo")
                .setCachingTolerance(0.01);
        rightButterflyServo = new ServoEx(hwMap, "Drivetrain rightButterflyServo")
                .setCachingTolerance(0.01);
        leftPto = new ServoEx(hwMap, "Drivetrain leftPto")
                .setCachingTolerance(0.01);
        rightPto = new ServoEx(hwMap, "Drivetrain rightPto")
                .setCachingTolerance(0.01);

        leftTouch = hwMap.get(TouchSensor.class, "Drivetrain leftTouch");
        rightTouch = hwMap.get(TouchSensor.class, "Drivetrain rightTouch");
        backTouch = hwMap.get(TouchSensor.class, "Drivetrain backTouch");
        frontTouch = hwMap.get(TouchSensor.class, "Drivetrain frontTouch");
    }

    public boolean pressed(){
        return (leftTouch.isPressed() || rightTouch.isPressed() || frontTouch.isPressed() || backTouch.isPressed());
    }

    public void update(){
        if (lowerWhenUntouchedTimer.done()) liftButterflyWheels();

        if (pressed()){
            lowerButterflyWheels();
            wasPressed = true;
        } else {
            if (wasPressed) lowerWhenUntouchedTimer.start();
            wasPressed = false;
        }
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
        if (!wheelsUp) {
            leftButterflyServo.set(LEFT_WHEEL_UP_POS);
            rightButterflyServo.set(RIGHT_WHEEL_UP_POS);
            wheelsUp = true;
        }
    }
    public void lowerButterflyWheels(){
        if (wheelsUp) {
            leftButterflyServo.set(LEFT_WHEEL_DOWN_POS);
            rightButterflyServo.set(RIGHT_WHEEL_DOWN_POS);
            wheelsUp = false;
        }
    }

    public void enablePto(){
        liftButterflyWheels();
        leftPto.set(LEFT_PTO_IN_POS);
        rightPto.set(RIGHT_PTO_IN_POS);
        ptoEnabled = true;
    }

    public void disablePto(){
        leftPto.set(LEFT_PTO_OUT_POS);
        rightPto.set(RIGHT_PTO_OUT_POS);
        ptoEnabled = false;
    }

    public Command runPto = sequential(
            instant(() -> {
                if (!ptoEnabled) enablePto();
                backLeft.set(1); //may need to reverse
                backRight.set(1);
            }),
            waitMs(1000), //TODO tune all waits
            instant(() -> {
                frontLeft.set(-1);
                frontRight.set(-1);
            }),
            waitMs(500),
            instant(() -> {
                backLeft.set(-1); //may need to reverse
                backRight.set(-1);
            }),
            waitMs(1000),
            instant(() -> {
                backLeft.set(0);
                backRight.set(0);
            })
    );
}
