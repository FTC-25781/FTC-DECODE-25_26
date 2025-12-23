package org.firstinspires.ftc.teamcode.layered.physical1;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class FlywheelMotor {
    public DcMotorEx flywheelShooter;
    private Telemetry telemetry;
    private VoltageSensor batteryVoltage;
    public static final double TICKS_PER_REV = 28.0;
    public static final double RPM_TOLERANCE = 50.0;
    private double targetRPM = 0;
    public static double kP = 10.10;  // Start here, increase if sluggish
    public static double kI = 0.0000;   // Keep small to prevent windup
    public static double kD = 5.40;   // Helps reduce overshoot
    public static double kF = 13.40;  // Most important - gets you close to target
    public static double LAUNCH_ANGLE_DEG = 51.0;
    public static double SHOOTER_HEIGHT_MM = 11.0 * 25.4;  // 279.4mm
    public static double TARGET_HEIGHT_MM = 43.0 * 25.4;   // 1092.2mm
    public static double WHEEL_DIAMETER_MM = 96.0;
    public static double VELOCITY_EFFICIENCY = 0.325;  // Empirically tuned (replaces 1.99x)
    public static double GRAVITY_MM = 9800.0;  //  mm/s²
    public static double RPM_CORRECTION_FACTOR = 1.172;
    public FlywheelMotor(HardwareMap hardwareMap, Telemetry telemetry) {
        this.flywheelShooter = hardwareMap.get(DcMotorEx.class, "dmot");
        this.telemetry = telemetry;
        this.batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        flywheelShooter.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        flywheelShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        updatePIDFCoefficients();
    }
    public void updatePIDFCoefficients() {
        PIDFCoefficients pidfNew = new PIDFCoefficients(kP, kI, kD, kF);
        flywheelShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        PIDFCoefficients pidfActual = flywheelShooter.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        telemetry.addData("PIDF Set", "P=%.1f, I=%.2f, D=%.1f, F=%.1f",
                pidfActual.p, pidfActual.i, pidfActual.d, pidfActual.f);
    }
    public double getVelocityEfficiency(double distanceMM) {
        if (distanceMM <= 2000) return 0.327;
        if (distanceMM >= 4500) return 0.36;

        // Linear interpolation
        double ratio = (distanceMM - 2000) / (4500 - 2000);
        return 0.327 + ratio * (0.36 - 0.327);
    }

    public void setRPM(double targetRPM) {
        this.targetRPM = targetRPM;

        if (targetRPM <= 0) {
            flywheelShooter.setPower(0);
            return;
        }

        // Convert RPM to ticks per second
        double targetTicksPerSecond = targetRPM * TICKS_PER_REV / 60.0;


        // Set velocity - PIDF controller handles the rest
        flywheelShooter.setVelocity(targetTicksPerSecond);

        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Target Ticks/Sec", "%.0f", targetTicksPerSecond);
        telemetry.addData("Battery Voltage", "%.1fV", batteryVoltage.getVoltage());
        telemetry.update();
    }

    public boolean isShooterReady() {
        double currentRPM = getCurrentRPM();
        double error = Math.abs(targetRPM - currentRPM);
        boolean ready = error < RPM_TOLERANCE && targetRPM > 0;

        telemetry.addData("RPM Error", "%.0f RPM", error);
        telemetry.addData("Shooter Ready", ready ? "YES" : "NO");

        return ready;
    }

    public double targetRPM(double distanceMM) {
        double calculatedRPM = calculateRequiredRPM(distanceMM);

        ///calculatedRPM *= RPM_CORRECTION_FACTOR;

        double clampedRPM = Range.clip(calculatedRPM, 0, 6000);

        telemetry.addData("Calc RPM (raw)", "%.0f", calculatedRPM);
        telemetry.addData("Target RPM (final)", "%.0f", clampedRPM);

        return clampedRPM;
    }

    public double getCurrentRPM() {
        double ticksPerSecond = flywheelShooter.getVelocity();
        return (ticksPerSecond * 60.0) / TICKS_PER_REV;
    }
    private double calculateRequiredRPM(double horizontalDistanceMM) {
        // Convert angle to radians
        double launchAngleRad = Math.toRadians(LAUNCH_ANGLE_DEG);

        // Calculate height difference
        double verticalDistanceMM = TARGET_HEIGHT_MM - SHOOTER_HEIGHT_MM;

        // Trig values
        double cosAngle = Math.cos(launchAngleRad);
        double tanAngle = Math.tan(launchAngleRad);

        // Projectile motion equation:
        // v₀ = sqrt( g * d² / (2 * cos²(θ) * (d * tan(θ) - h)) )
        double denominator = 2.0 * cosAngle * cosAngle *
                (horizontalDistanceMM * tanAngle - verticalDistanceMM);

        // Check for invalid trajectory
        if (denominator <= 0) {
            telemetry.addData("WARNING", "Invalid trajectory at %.0fmm", horizontalDistanceMM);
            return 0;
        }

        double numerator = GRAVITY_MM * horizontalDistanceMM * horizontalDistanceMM;

        // Calculate required ball exit velocity (mm/s)
        double requiredBallVelocity = Math.sqrt(numerator / denominator);

        double efficiency = getVelocityEfficiency(horizontalDistanceMM);
        double requiredWheelVelocity = requiredBallVelocity / efficiency;

        telemetry.addData("Velocity Efficiency", "%.3f", efficiency);

        // Convert to RPM: RPM = (velocity * 60) / (π * diameter)
        double wheelCircumference = Math.PI * WHEEL_DIAMETER_MM;
        double requiredRPM = (requiredWheelVelocity * 60.0) / wheelCircumference;

        telemetry.addData("Ball Velocity", "%.0f mm/s", requiredBallVelocity);
        telemetry.addData("Wheel Velocity", "%.0f mm/s", requiredWheelVelocity);

        return requiredRPM;
    }

    public void stop() {
        targetRPM = 0;
        flywheelShooter.setPower(0);
    }
}