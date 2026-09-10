package org.firstinspires.ftc.teamcode.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static java.lang.Math.max;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.Timing;

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
@Configurable
public class Drivetrain {
    public MotorEx frontLeft, frontRight, backLeft, backRight;
    private ServoEx leftButter, rightButter;
    public static double
            LEFT_WHEEL_UP_POS = 0.925,
            LEFT_WHEEL_DOWN_POS = 0.825,
            RIGHT_WHEEL_UP_POS = 0.85,
            RIGHT_WHEEL_DOWN_POS = 0.95,
            RIGHT_PTO_OUT_POS = 1,
            RIGHT_PTO_IN_POS = 0,
            LEFT_PTO_OUT_POS = 1,
            LEFT_PTO_IN_POS = 0,
            MAX_ACCEL = 10; //tune this

    public boolean wheelsUp = false;
    public boolean ptoEnabled = false;
    private Timing.Timer lowerWhenUntouchedTimer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
    private boolean wasPressed = false;
    private final double WIDTH = 17.7, LENGTH = 17.7;
//    public Follower follower;
    public boolean slowDrive = false;
    private final double SLOW_MODE_MULTIPLIER = 0.2;
    private final double FIELD_SIZE = 144; // inches — adjust to your field
    private final double WALL_MARGIN = 2;  // same threshold as your snippet
    private final double BUMP_MIN_X = 47.75, BUMP_MIN_Y = 53, BUMP_MAX_X = 96.25, BUMP_MAX_Y = 91;
    private double previousVelocity = 0;

    public enum Side { FRONT, RIGHT, BACK, LEFT }


    public void initialize(HardwareMap hwMap){
        frontLeft = new MotorEx(hwMap, "Drivetrain frontLeft").setCachingTolerance(0.005);
        frontRight = new MotorEx(hwMap, "Drivetrain frontRight").setCachingTolerance(0.005);
        backLeft = new MotorEx(hwMap, "Drivetrain backLeft").setCachingTolerance(0.005);
        backRight = new MotorEx(hwMap, "Drivetrain backRight").setCachingTolerance(0.005);

        frontLeft.setInverted(true);
        backLeft.setInverted(true);

        leftButter = new ServoEx(hwMap, "Drivetrain leftButter");
        rightButter = new ServoEx(hwMap, "Drivetrain rightButter");

//        follower = Constants.createFollower(hwMap);
    }

    public void update(){
//        follower.update();
//        if (follower.isBusy()) liftButterflyWheels();
    }

//    public void setPose(Pose pose){
//        follower.setPose(pose);
//    }
//    public void setStartingPose(Pose pose){
//        follower.setStartingPose(pose);
//    }



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
        double heading = /*follower.getPose().getHeading()*/0;

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

    public void autoLiftTouchSensor(){
        Map<Side, Boolean> sensorStates = new HashMap<>();
        sensorStates.put(Side.FRONT, false);
        sensorStates.put(Side.RIGHT, false);
        sensorStates.put(Side.BACK,  false);
        sensorStates.put(Side.LEFT,  false);

        if (hasUnexpectedPressedSensor(sensorStates)){
            lowerButterflyWheels();
            wasPressed = true;
        } else {
            if (lowerWhenUntouchedTimer.done()) liftButterflyWheels();
            if (wasPressed) lowerWhenUntouchedTimer.start();
            wasPressed = false;
        }
    }

    public void autoLiftVelocityMatch(double loopTime){
        if (false/*(follower.getVelocity.getMagnitude() - previousVelocity)/loopTime < MAX_ACCEL * */){
            lowerButterflyWheels();
            wasPressed = true;
        } else {
            if (lowerWhenUntouchedTimer.done()) liftButterflyWheels();
            if (wasPressed) lowerWhenUntouchedTimer.start();
            wasPressed = false;
        }
        /*previousVelocity = follower.getVelocity.getMagnitude();*/
    } //not done, probably won't be used


    public void driveButterfly(double forward, double turn){
        double left = -forward + turn;
        double right = -forward - turn;
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
//        if (wheelsUp) {
//            if (!follower.isTeleopDrive()) follower.startTeleOpDrive();
//            follower.setTeleOpDrive(forward, strafe, turn);
//        } else {
//            driveButterfly(forward, turn);
        driveStrafe(forward, strafe, turn);
        //}
    }

    /**
     * reverse C shape
     * (this was vibecoded)
     * @return the poses of the corners
     */
    public Pose[] getRobotCorners() {
        Pose robotPose = new Pose()/*follower.getPose()*/;
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
            wheelsUp = true;
            leftButter.set(LEFT_WHEEL_UP_POS);
            rightButter.set(RIGHT_WHEEL_UP_POS);
        }
    }
    public void lowerButterflyWheels(){
        if (wheelsUp) {
            wheelsUp = false;
            leftButter.set(LEFT_WHEEL_DOWN_POS);
            rightButter.set(RIGHT_WHEEL_DOWN_POS);
        }
    }

    public void enablePto(){
        liftButterflyWheels();
        ptoEnabled = true;
    }

    public void disablePto(){
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

    public void setIndividualPowers(int which, double power){
        if (which == 1){
            frontRight.set(power);
        }

        if (which == 2){
            frontLeft.set(power);
        }

        if (which == 3){
            backRight.set(power);
        }

        if (which == 4){
            backLeft.set(power);
        }
    }

    public void driveStrafe(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeft.set(maxSpeed * (frontLeftPower / maxPower));
        frontRight.set(maxSpeed * (frontRightPower / maxPower));
        backLeft.set(maxSpeed * (backLeftPower / maxPower));
        backRight.set(maxSpeed * (backRightPower / maxPower));
    }
}
