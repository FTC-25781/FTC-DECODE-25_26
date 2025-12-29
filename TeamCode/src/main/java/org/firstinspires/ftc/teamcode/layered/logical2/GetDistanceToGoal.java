package org.firstinspires.ftc.teamcode.layered.logical2;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class calculates the straight-line distance from the robot to the goal
 * based on the robot's current position on the field.
 */
public class GetDistanceToGoal {

    Follower follower; // The Pedro Pathing follower that tracks robot position (X, Y)
    boolean isBlue;    // Flag to determine which side of the field (and which goal) to target
    double distance;
    Telemetry telemetry;// The calculated distance to the goal

    /**
     * Constructor for the distance calculator.
     * @param follower The localization engine (Pedro Pathing)
     * @param isBlue   True if playing on the Blue alliance, False for Red
     */
    public GetDistanceToGoal(Follower follower, boolean isBlue, Telemetry telemetry) {
        this.follower = follower;
        this.isBlue = isBlue;
        this.telemetry = telemetry;
    }

    /**
     * Calculates the 2D distance to the center of the goal.
     * @return The distance to the goal in millimeters (mm).
     */
    public double getDistanceToGoal(){
        // Get the current estimated position (Pose) of the robot from the follower
        Pose robotPose = this.follower.getPose();
        double goalX;

        // Define Goal X-coordinate based on Alliance color.
        // In FTC, the field is usually 144x144 inches.
        if(isBlue) {
            // Blue Goal is located near the X=12 inch mark
            goalX = 12;
        } else {
            // Red Goal is located near the X=132 inch mark (opposite side)
            goalX = 132;
        }

        // The Y-coordinate for the high goals is typically at the back of the field (132 inches)
        double goalY = 132;

        // Calculate the difference in X and Y (the sides of a right triangle)
        double dx = goalX - robotPose.getX();
        double dy = goalY - robotPose.getY();

        telemetry.addData("GoalX:", goalX);
        telemetry.addData("GoalY:", goalY);
        telemetry.addData("dX:", dx);
        telemetry.addData("dY:", dy);
        telemetry.update();

        // Use the Pythagorean Theorem: Distance = sqrt(dx² + dy²)
        // This gives the "as-the-crow-flies" distance in inches.
        distance  = Math.sqrt(dx * dx + dy * dy);

        // The physics engine (FlywheelMotor) requires millimeters.
        // 1 inch = 25.4 millimeters.
        return distance * 25.4;
    }

    double getDistanceInInches()
    {
        return distance;
    }
}
