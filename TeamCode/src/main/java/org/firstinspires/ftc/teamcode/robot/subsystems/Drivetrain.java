package org.firstinspires.ftc.teamcode.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static java.lang.Math.max;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.robot.pedroPathing.Constants;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
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
    private final double WIDTH = 17, LENGTH = 17;
    public Follower follower;
    public boolean slowDrive = false;
    private final double SLOW_MODE_MULTIPLIER = 0.2;
    public static final double FIELD_SIZE = 144; // inches — adjust to your field
    public static final double WALL_MARGIN = 2;  // same threshold as your snippet

    public enum Side { FRONT, RIGHT, BACK, LEFT }


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
        follower = Constants.createFollower(hwMap);
    }

    public void update(){
        follower.update();
        if (follower.isBusy()) liftButterflyWheels();
    }

    public void setPose(Pose pose){
        follower.setPose(pose);
    }
    public void setStartingPose(Pose pose){
        follower.setStartingPose(pose);
    }



    // Your existing check, generalized to all four walls
    public boolean isTouchingWall() {
        for (Pose corner : getRobotCorners()) {
            if (corner.getX() < WALL_MARGIN || corner.getX() > FIELD_SIZE - WALL_MARGIN ||
                    corner.getY() < WALL_MARGIN || corner.getY() > FIELD_SIZE - WALL_MARGIN) {
                return true;
            }
        }
        return false;
    }
    // Returns every side that SHOULD be touching a wall, based on geometry
    public Set<Side> getExpectedTouchedSides() {
        Set<Side> touchedSides = new HashSet<>();
        Pose[] corners = getRobotCorners();
        double heading = follower.getPose().getHeading();

        double frontNx = Math.cos(heading),  frontNy = Math.sin(heading);
        double rightNx = Math.sin(heading),  rightNy = -Math.cos(heading);
        double backNx  = -frontNx,           backNy  = -frontNy;
        double leftNx  = -rightNx,           leftNy  = -rightNy;

        for (Pose corner : corners) {
            double wallNx = 0, wallNy = 0;
            boolean hitWall = false;

            if (corner.getX() < WALL_MARGIN)                    { wallNx -= 1; hitWall = true; }
            else if (corner.getX() > FIELD_SIZE - WALL_MARGIN)  { wallNx += 1; hitWall = true; }

            if (corner.getY() < WALL_MARGIN)                    { wallNy -= 1; hitWall = true; }
            else if (corner.getY() > FIELD_SIZE - WALL_MARGIN)  { wallNy += 1; hitWall = true; }

            if (!hitWall) continue;

            // A side "counts" as touching if its outward normal has a positive
            // component along the wall normal (i.e. it's facing into the wall)
            if (frontNx * wallNx + frontNy * wallNy > 0.01) touchedSides.add(Side.FRONT);
            if (rightNx * wallNx + rightNy * wallNy > 0.01) touchedSides.add(Side.RIGHT);
            if (backNx  * wallNx + backNy  * wallNy > 0.01) touchedSides.add(Side.BACK);
            if (leftNx  * wallNx + leftNy  * wallNy > 0.01) touchedSides.add(Side.LEFT);
        }

        return touchedSides;
    }

    // Checks if any PRESSED sensor is on a side that shouldn't be touching a wall
    public boolean hasUnexpectedPressedSensor(Map<Side, Boolean> sensorStates) {
        Set<Side> expectedTouched = getExpectedTouchedSides();

        for (Map.Entry<Side, Boolean> entry : sensorStates.entrySet()) {
            Side side = entry.getKey();
            boolean isPressed = entry.getValue();

            if (isPressed && !expectedTouched.contains(side)) {
                return true; // this sensor is pressed but geometry says it shouldn't be
            }
        }
        return false;
    }

    public void autoLift(){
        Map<Side, Boolean> sensorStates = new HashMap<>();
        sensorStates.put(Side.FRONT, frontTouch.isPressed());
        sensorStates.put(Side.RIGHT, rightTouch.isPressed());
        sensorStates.put(Side.BACK,  backTouch.isPressed());
        sensorStates.put(Side.LEFT,  leftTouch.isPressed());

        if (hasUnexpectedPressedSensor(sensorStates)){
            lowerButterflyWheels();
            wasPressed = true;
        } else {
            if (lowerWhenUntouchedTimer.done()) liftButterflyWheels();
            if (wasPressed) lowerWhenUntouchedTimer.start();
            wasPressed = false;
        }
    }

    public void driveButterfly(double forward, double turn){
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

    public void drive(double forward, double strafe, double turn){
        if (slowDrive) {
            forward *= SLOW_MODE_MULTIPLIER;
            strafe *= SLOW_MODE_MULTIPLIER;
            turn *= SLOW_MODE_MULTIPLIER;
        }
        if (wheelsUp) {
            if (!follower.isTeleopDrive()) follower.startTeleOpDrive();
            follower.setTeleOpDrive(forward, strafe, turn);
        } else {
            driveButterfly(forward, turn);
        }
    }

    /**
     * reverse C shape
     * (this was vibecoded)
     * @return the poses of the corners
     */
    public Pose[] getRobotCorners() {
        Pose robotPose = follower.getPose();
        double half = 9.0; // half of 18 inches
        double x = robotPose.getX();
        double y = robotPose.getY();
        double heading = robotPose.getHeading();

        // Corners relative to center, in robot-local frame (x forward, y left)
        double[][] localCorners = {
                { half,  half}, // front-left
                { half, -half}, // front-right
                {-half, -half}, // back-right
                {-half,  half}  // back-left
        };

        Pose[] corners = new Pose[4];
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        for (int i = 0; i < 4; i++) {
            double lx = localCorners[i][0];
            double ly = localCorners[i][1];

            // Rotate local offset into field frame
            double fieldX = x + (lx * cos - ly * sin);
            double fieldY = y + (lx * sin + ly * cos);

            corners[i] = new Pose(fieldX, fieldY, heading);
        }
        return corners;
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
