package org.firstinspires.ftc.teamcode.layered.logical2;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BallisticsCalculator {

    // TUNABLE PARAMETERS - adjust these for your robot
    public static double LAUNCH_ANGLE_DEG = 51.0;      // Launch angle in degrees
    public static double SHOOTER_HEIGHT = 11.0*25.4;  // Height of shooter off ground
    public static double TARGET_HEIGHT = 43*25.4;   // Height of target basket
    public static double WHEEL_DIAMETER = 96;   // Diameter of flywheel
    public static double VELOCITY_EFFICIENCY = 0.75;    // Ball exit velocity / wheel velocity (0.7-0.9 typical)
    public static double GRAVITY = 9800;              // Gravity in inches/s² (32.185 ft/s² = 386.22 in/s²)

    /**
     * Calculate required RPM based on horizontal distance to target
     * @param horizontalDistance Distance to target in inches
     * @return Required flywheel RPM
     */
    public static double calculateRequiredRPM(double horizontalDistance) {
        // Convert angle to radians
        double launchAngleRad = Math.toRadians(LAUNCH_ANGLE_DEG);

        // Calculate height difference
        double heightDifference = TARGET_HEIGHT - SHOOTER_HEIGHT;

        // Projectile motion equation for initial velocity:
        // v₀ = sqrt( g * d² / (2 * cos²(θ) * (d * tan(θ) - h)) )
        // where: g = gravity, d = horizontal distance, θ = launch angle, h = height difference

        double cosAngle = Math.cos(launchAngleRad);
        double tanAngle = Math.tan(launchAngleRad);

        // Calculate denominator: 2 * cos²(θ) * (d * tan(θ) - h)
        double denominator = 2.0 * cosAngle * cosAngle *
                (horizontalDistance * tanAngle - heightDifference);

        // Check for invalid trajectory (target below/behind shooter with this angle)
        if (denominator <= 0) {
            return 0; // Invalid trajectory
        }

        // Calculate numerator: g * d²
        double numerator = GRAVITY * horizontalDistance * horizontalDistance;

        // Calculate required ball exit velocity (inches/second)
        double requiredBallVelocity = Math.sqrt(numerator / denominator);

        // Account for energy loss: wheel needs to spin faster than ball exit velocity
        double requiredWheelVelocity = requiredBallVelocity / VELOCITY_EFFICIENCY;

        // Convert wheel velocity to RPM
        // Wheel circumference = π * diameter
        // Velocity (mm/s) = RPM/60 * circumference
        // Therefore: RPM = (velocity * 60) / (π * diameter)
        double wheelCircumference = Math.PI * WHEEL_DIAMETER;
        double requiredRPM = (requiredWheelVelocity * 60.0) / wheelCircumference;

        return requiredRPM;
    }
    // Coverts RPM to angular Velocity in rad/sec
    public static double calculateRange(double rpm) {
        // Convert RPM to wheel velocity
        double wheelCircumference = Math.PI * WHEEL_DIAMETER;
        double wheelVelocity = (rpm * wheelCircumference) / 60.0;

        // Account for efficiency
        double ballVelocity = wheelVelocity * VELOCITY_EFFICIENCY;

        // Convert angle to radians
        double launchAngleRad = Math.toRadians(LAUNCH_ANGLE_DEG);
        double heightDifference = TARGET_HEIGHT - SHOOTER_HEIGHT;

        // Projectile range equation:
        // R = (v₀² / g) * (sin(2θ) + sqrt(sin²(2θ) + (2gh/v₀²)))
        double v0Squared = ballVelocity * ballVelocity;
        double sin2Theta = Math.sin(2 * launchAngleRad);

        double term1 = sin2Theta;
        double term2 = Math.sqrt(sin2Theta * sin2Theta +
                (2 * GRAVITY * heightDifference / v0Squared));

        double range = (v0Squared / GRAVITY) * (term1 + term2);

        return range;
    }

    /**
     * Get maximum theoretical range for current settings
     * @return Maximum range in inches
     */
    public static double getMaximumRange() {
        // At 45° with no height difference, max range = v₀² / g
        // With height difference, it's more complex
        double maxRPM = 6000; // Assume max motor RPM
        return calculateRange(maxRPM);
    }

    /**
     * Validate if a shot is possible with current settings
     * @param horizontalDistance Distance to target
     * @return true if shot is theoretically possible
     */
    public static boolean isShotPossible(double horizontalDistance) {
        double requiredRPM = calculateRequiredRPM(horizontalDistance);
        return requiredRPM > 0 && requiredRPM <= 6000; // Check if within motor limits
    }

    /**
     * Get diagnostic information for tuning
     * @param distance Distance to target
     * @return Diagnostic string
     */
    public static String getDiagnostics(double distance) {
        double rpm = calculateRequiredRPM(distance);
        double actualRange = calculateRange(rpm);
        double error = Math.abs(actualRange - distance);

        return String.format(
                "Distance: %.1f in | Required RPM: %.0f | Actual Range: %.1f in | Error: %.1f in",
                distance, rpm, actualRange, error
        );
    }
}