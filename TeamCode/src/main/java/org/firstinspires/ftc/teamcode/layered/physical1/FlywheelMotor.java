package org.firstinspires.ftc.teamcode.layered.physical1;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class FlywheelMotor {
    public DcMotorEx flywheelShooter;
    Telemetry telemetryF;
    public VoltageSensor batteryVoltage;

    // Motor specs
    public static final double TICKS_PER_REV = 28.0;
    public static final double RPM_TOLERANCE = 50.0;
    public double targetRPM;
    public static double kS_TICKS = 120;
    public static double kV_TICKS = 0.02;
    public static double NOMINAL_VOLTAGE = 13.5;

    public static double LAUNCH_ANGLE_DEG = 51.0;      // Launch angle in degrees
    public static double SHOOTER_HEIGHT = 11.0*25.4;  // Height of shooter off ground
    public static double TARGET_HEIGHT = 43*25.4;   // Height of target basket
    public static double WHEEL_DIAMETER = 96;   // Diameter of flywheel
    public static double VELOCITY_EFFICIENCY = 0.75;    // Ball exit velocity / wheel velocity (0.7-0.9 typical)
    public static double GRAVITY = 9800;              // Gravity in inches/s² (32.185 ft/s² = 386.22 in/s²)

//    public static double kP = 9.25;
//    public static double kI = 2.2;
//    public static double kD = 7.0;
//    public static double kF = 11.7;

    public FlywheelMotor(HardwareMap hardwareMap, Telemetry telemetry) {
        flywheelShooter = hardwareMap.get(DcMotorEx.class, "dmot");
        telemetryF = telemetry;
        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        flywheelShooter.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Use RUN_USING_ENCODER like your working test code
        // flywheelShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        // updatePIDFCoefficients();
    }

    public void setRPM(double targetRPM) {
        this.targetRPM = targetRPM;

        // Convert RPM to ticks per seconds
        double targetTicksPerSecond = targetRPM * TICKS_PER_REV / 60.0;

        double feedforwardTicks = kS_TICKS + kV_TICKS * targetTicksPerSecond;
        targetTicksPerSecond += feedforwardTicks;

        double voltageCompensation = NOMINAL_VOLTAGE / batteryVoltage.getVoltage();
        targetTicksPerSecond *= voltageCompensation;

        flywheelShooter.setVelocity(targetTicksPerSecond);
        telemetryF.addData("Target Ticks/Sec", targetTicksPerSecond);
    }

    public boolean isShooterReady() {
        return Math.abs(targetRPM - getCurrentRPM()) < RPM_TOLERANCE;
    }

    public double targetRPM(double distance) {
        double calculatedRPM = calculateRequiredRPM(distance);
        calculatedRPM *= 1.99;
        return Range.clip(calculatedRPM, 0, 6000);
    }

    public double getCurrentRPM() {
        // Convert ticks/sec back to RPM
        double ticksPerSecond = flywheelShooter.getVelocity();
        return ticksPerSecond * 60.0 / TICKS_PER_REV;
    }

    public double calculateRequiredRPM(double horizontalDistance) {
        // Convert angle to radians
        double launchAngleRad = Math.toRadians(LAUNCH_ANGLE_DEG);

        // Calculate height difference
        double verticalDistance = TARGET_HEIGHT - SHOOTER_HEIGHT;

        // Projectile motion equation for initial velocity:
        // v₀ = sqrt( g * d² / (2 * cos²(θ) * (d * tan(θ) - h)) )
        // where: g = gravity, d = horizontal distance, θ = launch angle, h = height difference

        double cosAngle = Math.cos(launchAngleRad);
        double tanAngle = Math.tan(launchAngleRad);

        // Calculate denominator: 2 * cos²(θ) * (d * tan(θ) - h)
        double denominator = 2.0 * cosAngle * cosAngle *
                (horizontalDistance * tanAngle - verticalDistance);

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
}