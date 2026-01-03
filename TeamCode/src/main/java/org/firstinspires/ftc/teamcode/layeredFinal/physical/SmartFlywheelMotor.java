package org.firstinspires.ftc.teamcode.layeredFinal.physical;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Physical layer wrapper for the flywheel motor.
 * Uses built-in velocity control (PIDF) to maintain consistent RPMs,
 * which is essential for shot repeatability.
 */
public class SmartFlywheelMotor {
    private DcMotorEx flywheelMotor;

    // The desired speed we want the motor to reach and maintain
    double currTargetVelocity = 0;

    /**
     * PIDF Constants:
     * P (Proportional) handles the immediate response to error.
     * F (Feedforward) provides the base power needed to maintain a theoretical speed.
     * These were tuned specifically for this motor/gearbox combo.
     */
    final double P = 342;
    final double F = 14;

    public SmartFlywheelMotor(HardwareMap hardwareMap) {
        // "dmot" must match the name in the Driver Station configuration
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "dmot");

        // RUN_USING_ENCODER is required for the internal PID controller to work
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Reverse direction if the motor is mounted backward relative to the shooter's needs
        flywheelMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Apply custom tuning values.
        // We leave I and D at 0 unless "hunting" or "oscillations" become an issue.
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    /**
     * Sends the target velocity command to the motor controller.
     * This should be called repeatedly in the main loop.
     */
    public void update() {
        flywheelMotor.setVelocity(currTargetVelocity);
    }

    /**
     * Updates the internal target velocity.
     * @param vel The new target speed (typically in encoder ticks per second).
     */
    public void setVelocity(double vel) {
        currTargetVelocity = vel;
    }

    /**
     * Reads the actual speed of the motor from the encoder.
     * Used by the State Machine to ensure the shooter is "Ready" before firing.
     * @return Current velocity in ticks per second.
     */
    public double getCurVelocity() {
        return flywheelMotor.getVelocity();
    }

    /**
     * Calculates the difference between target and actual speed.
     * High error means the motor is still spooling up or is jammed.
     * @return The velocity gap (Target - Actual).
     */
    public double getError() {
        return currTargetVelocity - getCurVelocity();
    }
}
