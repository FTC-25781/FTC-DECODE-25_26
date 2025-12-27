package org.firstinspires.ftc.teamcode.layered.logical2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.layered.control3.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.layered.physical1.FlywheelMotor;

@TeleOp(name="Angular PID")
public class PIDShooter extends LinearOpMode {
    // --- Subsystems ---
    public FlywheelMotor flywheel;             // Our physics-based motor controller
    public GetDistanceToGoal distanceCalculation; // Logic to find the distance between robot and goal
    public Follower follower;                  // Pedro Pathing follower for localization (X, Y, Heading)

    // Starting position of the robot on the field (inches and radians)
    public Pose startingPose = new Pose(72, 72, Math.toRadians(135));

    @Override
    public void runOpMode() {
        // Step 1: Initialize hardware and localization
        initHardware();

        // Step 2: Wait for the driver to press the "Play" button
        waitForStart();

        // Step 3: Main Loop
        while (opModeIsActive()) {
            // Update the Pedro Pathing follower to get the latest X, Y coordinates
            follower.update();

            // Calculate the current distance from the robot to the goal (likely using Pythagoras internally)
            double distance = distanceCalculation.getDistanceToGoal();

            // Pass the distance into our physics engine to get the required motor speed
            double targetRPM = flywheel.targetRPM(distance);

            // Command the motor to reach that specific RPM
            flywheel.setRPM(targetRPM);

            // --- Telemetry (Debugging data sent to the Driver Station) ---
            telemetry.addData("Current Position-x:", follower.getPose().getX());
            telemetry.addData("Current Position-Y:", follower.getPose().getY());
            telemetry.addData("Current Heading:", follower.getPose().getHeading());

            telemetry.addData("Distance (inches):", distanceCalculation.getDistanceInInches());
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", flywheel.getCurrentRPM());

            // Check raw velocity and power to ensure the PIDF is working correctly
            telemetry.addData("Velocity (rad/s)", flywheel.flywheelShooter.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("Motor Power", flywheel.flywheelShooter.getPower());

            // 'Ready' returns true if the current RPM is within the tolerance of target RPM
            telemetry.addData("Ready", flywheel.isShooterReady());
            telemetry.update();
        }

        // Step 4: Safety shutdown when the OpMode is stopped
        flywheel.stop();
    }

    /**
     * Set up the hardware and link the subsystems together.
     */
    public void initHardware() {
        // Initialize the flywheel controller
        flywheel = new FlywheelMotor(hardwareMap, telemetry);

        // Initialize Pedro Pathing using your custom constants
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        // Initialize the distance calculator, linking it to the follower's position
        // The 'true' likely indicates a specific goal side or calculation mode
        distanceCalculation = new GetDistanceToGoal(follower, true,telemetry);
    }
}