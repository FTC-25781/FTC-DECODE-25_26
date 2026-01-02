package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartFlywheelMotor;

/**
 * The Flywheel class provides high-level control over the robot's shooter motor.
 * It manages velocity presets for different shooting distances and handles
 * fine-tuning adjustments during matches.
 */
public class Flywheel {
    // Reference to the physical motor wrapper that likely handles PID/Feedforward
    SmartFlywheelMotor flywheel;

    // Default velocity presets (Ticks per second or RPM depending on implementation)
    public double highVelocity = 1525; // Target for the far goal (tip of small triangle)
    public double lowVelocity = 1350;  // Target for the near goal (tip of big triangle)

    /**
     * Initializes the flywheel hardware.
     * @param hardwareMap The OpMode hardwareMap to locate the motor.
     */
    public Flywheel(HardwareMap hardwareMap) {
        flywheel = new SmartFlywheelMotor(hardwareMap);
    }

    /**
     * Sets the motor to the lower velocity preset, intended for closer targets.
     */
    public void setVelForCloseTip() {
        flywheel.setVelocity(lowVelocity);
    }

    /**
     * Sets the motor to the higher velocity preset, intended for further targets.
     */
    public void setVelForFarTip() {
        flywheel.setVelocity(highVelocity);
    }

    /**
     * Increases or decreases the far-range velocity preset.
     * Useful for on-the-fly battery compensation or range adjustments.
     * @param amount The value to add to the current highVelocity.
     */
    public void updateHighVelocity(double amount) {
        highVelocity += amount;
    }

    /**
     * Increases or decreases the near-range velocity preset.
     * @param amount The value to add to the current lowVelocity.
     */
    public void updateLowVelocity(double amount) {
        lowVelocity += amount;
    }

    /**
     * Immediately sets the target velocity to zero.
     */
    public void stopFlywheel() {
        flywheel.setVelocity(0.0);
    }

    /**
     * Manually sets the flywheel to a specific target velocity.
     * @param vel Target speed.
     */
    public void setVelocity(double vel) {
        flywheel.setVelocity(vel);
    }

    /**
     * Gets the current actual velocity from the motor encoders.
     * @return The current speed of the flywheel.
     */
    public double getVelocity() {
        return flywheel.getCurVelocity();
    }

    /**
     * Sets the motor to a reverse or specific speed to assist
     * with receiving rings from the human player.
     */
    public void humanPlayer() {
        // TODO: Change with real vel once tested
        flywheel.setVelocity(-0.0);
    }

    /**
     * Essential update loop. This should be called every cycle in the OpMode
     * to allow the underlying motor controller (SmartFlywheelMotor) to run its PID logic.
     */
    public void update() {
        flywheel.update();
    }
}
