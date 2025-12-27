package org.firstinspires.ftc.teamcode.layered.physical1;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * FlywheelMotor handles the physics-based velocity control for a shooter mechanism.
 * It uses projectile motion equations to determine the necessary RPM based on distance.
 */
public class FlywheelMotor {
    // Hardware components
    public DcMotorEx flywheelShooter;
    private Telemetry telemetry;
    private VoltageSensor batteryVoltage;

    // --- Physical Motor Constants ---
    public static final double TICKS_PER_REV = 28.0;   // Encoder ticks for one full revolution
    public static final double RPM_TOLERANCE = 75.0;  // Allowed error margin before considering the shooter "ready"
    private double targetRPM = 0;

    // --- PIDF Coefficients ---
    // These control how the motor reaches and maintains its speed.
    // kF (Feedforward) is the primary driver for high-velocity flywheels.
    public static double kP = 0.0;
    public static double kI = 0.0000;
    public static double kD = 0.0;
    public static double kF = 11.60;

    // --- Trajectory & Environment Constants ---
    public static final double LAUNCH_ANGLE_DEG = 35.0;      // Fixed angle of the shooter's exit ramp
    public static final double SHOOTER_HEIGHT_MM = 7.0*25.4;    // 7.0 inches converted to mm
    public static final double TARGET_HEIGHT_MM = 43*25.4;    // 43.0 inches converted to mm
    public static final double WHEEL_DIAMETER_MM = 96.0;     // Diameter of the flywheel (e.g., compliant wheel)
    public static final double GRAVITY_MM = 9800.0;          // Acceleration due to gravity in mm/s²

    /**
     * Constructor: Initializes the hardware and configures motor behavior.
     */
    public FlywheelMotor(HardwareMap hardwareMap, Telemetry telemetry) {
        this.flywheelShooter = hardwareMap.get(DcMotorEx.class, "dmot");
        this.telemetry = telemetry;
        // Accessing the battery sensor to monitor voltage fluctuations
        this.batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        flywheelShooter.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // RUN_USING_ENCODER enables the internal PIDF loop on the motor controller
        flywheelShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // FLOAT ensures the motor coasts to a stop rather than braking abruptly, protecting the motor/gears
        flywheelShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // updatePIDFCoefficients();
    }

    /**
     * Applies the defined PIDF coefficients to the motor controller.
     */
//    public void updatePIDFCoefficients() {
//        PIDFCoefficients pidfNew = new PIDFCoefficients(kP, kI, kD, kF);
//        flywheelShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);
//
//        // Confirmation of settings via telemetry
//        PIDFCoefficients pidfActual = flywheelShooter.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        telemetry.addData("PIDF Applied", "P=%.1f, I=%.2f, D=%.1f, F=%.1f",
//                pidfActual.p, pidfActual.i, pidfActual.d, pidfActual.f);
//    }

    /**
     * Accounts for "Slip" or efficiency loss between the wheel and the ball.
     * Efficiency often increases slightly with distance as the motor maintains higher momentum.
     */
    public double getVelocityEfficiency(double distanceMM) {
        if (distanceMM <= 2000) return 0.327;
        if (distanceMM >= 4500) return 0.36;

        // Linear interpolation for smooth efficiency scaling between 2 and 4.5 meters
        double ratio = (distanceMM - 2000) / (4500 - 2000);
        return 0.327 + ratio * (0.36 - 0.327);
    }

    /**
     * Commands the motor to spin at a specific Rotations Per Minute (RPM).
     */
    public void setRPM(double targetRPM) {
        this.targetRPM = targetRPM;

        if (targetRPM <= 0) {
            flywheelShooter.setPower(0);
            return;
        }

        // Convert RPM to encoder Ticks Per Second for the SDK's setVelocity method
        double targetTicksPerSecond = targetRPM * TICKS_PER_REV / 60.0;
        flywheelShooter.setVelocity(targetTicksPerSecond);

        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Battery Voltage", "%.1fV", batteryVoltage.getVoltage());
        telemetry.update();
    }

    /**
     * Returns true if the flywheel speed is within the allowed tolerance of the target.
     */
    public boolean isShooterReady() {
        double currentRPM = getCurrentRPM();
        double error = Math.abs(targetRPM - currentRPM);
        boolean ready = error < RPM_TOLERANCE && targetRPM > 0;

        telemetry.addData("RPM Error", "%.0f RPM", error);
        telemetry.addData("Ready to Shoot", ready);

        return ready;
    }

    /**
     * Calculates and returns the required RPM for a given distance, clamped to safe motor limits.
     */
    public double targetRPM(double distanceMM) {
        double calculatedRPM = calculateRequiredRPM(distanceMM);

        // Ensure the code doesn't try to exceed the motor's physical RPM limit (approx 6000 for many FTC motors)
        double clampedRPM = Range.clip(calculatedRPM, 0, 6000);

        telemetry.addData("Calculated RPM", "%.0f", clampedRPM);
        return clampedRPM;
    }

    /**
     * Calculates the current RPM based on real-time encoder feedback.
     */
    public double getCurrentRPM() {
        double ticksPerSecond = flywheelShooter.getVelocity();
        // Negative sign used to match motor orientation if necessary
        return -(ticksPerSecond * 60.0) / TICKS_PER_REV;
    }

    /**
     * Uses the Projectile Motion Kinematic Equation to find the launch velocity.
     * This considers the 2D plane (Distance vs Height).
     */
    private double calculateRequiredRPM(double horizontalDistanceMM) {
        double launchAngleRad = Math.toRadians(LAUNCH_ANGLE_DEG);
        double verticalDistanceMM = TARGET_HEIGHT_MM - SHOOTER_HEIGHT_MM;

        double cosAngle = Math.cos(launchAngleRad);
        double tanAngle = Math.tan(launchAngleRad);

        // Physics: v0 = sqrt( (g * d^2) / (2 * cos(theta)^2 * (d * tan(theta) - h)) )
        double denominator = 2.0 * (cosAngle * cosAngle) *
                (horizontalDistanceMM * tanAngle - verticalDistanceMM);

        // Logic check: if the target is out of range, the denominator becomes negative
        if (denominator <= 0) {
            telemetry.addData("WARNING", "Trajectory Impossible");
            return 0;
        }

        double numerator = GRAVITY_MM * Math.pow(horizontalDistanceMM, 2);
        double requiredBallVelocity = Math.sqrt(numerator / denominator);

        // Adjust ball velocity to required wheel tangential velocity based on friction/slip
        double efficiency = getVelocityEfficiency(horizontalDistanceMM);
        double requiredWheelVelocity = requiredBallVelocity / efficiency;

        // Convert tangential velocity (linear) to angular velocity (RPM)
        // Velocity = (RPM * Pi * Diameter) / 60
        double wheelCircumference = Math.PI * WHEEL_DIAMETER_MM;
        double requiredRPM = (requiredWheelVelocity * 60.0) / wheelCircumference;

        return requiredRPM;
    }

    /**
     * Emergency stop or end-of-match shutdown.
     */
    public void stop() {
        targetRPM = 0;
        flywheelShooter.setPower(0);
    }
}