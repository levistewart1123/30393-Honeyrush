package org.firstinspires.ftc.teamcode.tests;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.junit.jupiter.api.Test;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class RobotTests {

    @Test
    public void testTest(){
        int sum = 2 + 2;
        assertEquals(4, sum);
    }

    boolean butterDown = false;
    public static final double FIELD_SIZE = 144; // inches — adjust to your field
    public static final double WALL_MARGIN = 2;  // same threshold as your snippet

    @Test
    public void cornersTest(){
        Map<Drivetrain.Side, Boolean> sensorStates = new HashMap<>();
        sensorStates.put(Drivetrain.Side.FRONT, true);
        sensorStates.put(Drivetrain.Side.RIGHT, true);
        sensorStates.put(Drivetrain.Side.BACK,  true);
        sensorStates.put(Drivetrain.Side.LEFT,  false);

        if (hasUnexpectedPressedSensor(sensorStates)){
            butterDown = true;
        } else {
            butterDown = false;
        }
        assertFalse(butterDown);
        assertFalse(hasUnexpectedPressedSensor(sensorStates));
    }

    public boolean hasUnexpectedPressedSensor(Map<Drivetrain.Side, Boolean> sensorStates) {
        Set<Drivetrain.Side> expectedTouched = getExpectedTouchedSides();

        for (Map.Entry<Drivetrain.Side, Boolean> entry : sensorStates.entrySet()) {
            Drivetrain.Side side = entry.getKey();
            boolean isPressed = entry.getValue();

            if (isPressed && !expectedTouched.contains(side)) {
                return true; // this sensor is pressed but geometry says it shouldn't be
            }
        }
        return false;
    }

    public Set<Drivetrain.Side> getExpectedTouchedSides() {
        Set<Drivetrain.Side> touchedSides = new HashSet<>();
        Pose[] corners = getRobotCorners();
        double heading = new Pose(1, 1.5, 0).getHeading();

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
            if (frontNx * wallNx + frontNy * wallNy > 0.01) touchedSides.add(Drivetrain.Side.FRONT);
            if (rightNx * wallNx + rightNy * wallNy > 0.01) touchedSides.add(Drivetrain.Side.RIGHT);
            if (backNx  * wallNx + backNy  * wallNy > 0.01) touchedSides.add(Drivetrain.Side.BACK);
            if (leftNx  * wallNx + leftNy  * wallNy > 0.01) touchedSides.add(Drivetrain.Side.LEFT);
        }

        return touchedSides;
    }

    public Pose[] getRobotCorners() {
        Pose robotPose = new Pose(1, 1.5, 0);
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

    public Vector getFieldRelativeMovement(double forward, double strafe, double heading) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double fieldX = forward * cos - strafe * sin;
        double fieldY = forward * sin + strafe * cos;

        Vector movement = new Vector();
        movement.setOrthogonalComponents(fieldX, fieldY);
        return movement;
    }

    public Vector getRobotRelativeMovement(Vector fieldMovement, double heading) {
        double fieldX = fieldMovement.getXComponent();
        double fieldY = fieldMovement.getYComponent();

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        // Inverse rotation (rotate by -heading)
        double localForward = fieldX * cos + fieldY * sin;
        double localStrafe  = -fieldX * sin + fieldY * cos;

        Vector local = new Vector();
        local.setOrthogonalComponents(localForward, localStrafe);
        return local;
    }

    public void drive(){
        double forward = 1, strafe = 0,  turn = 0;
        Pose botPose = new Pose(72, 40, 45);
        if (false) {
            forward *= 0.2;
            strafe *= 0.2;
            turn *= 0.2;
        }


    }

//    public boolean isCloseToBump(){
//
//    }

}
