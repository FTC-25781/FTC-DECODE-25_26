package org.firstinspires.ftc.teamcode.layeredFinal.control;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.layeredFinal.logical.Flywheel;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.TransferColorSensor;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.TransferServos;

/**
 * The Transfer class coordinates the hardware components (flywheel, servos, sensors)
 * and provides a high-level API for the robot's shooting sequence.
 */
public class Transfer {

    // Low-level hardware wrappers
    private final TransferServos kickers;
    private final Flywheel shooter;
    private final TransferColorSensor colorSensors;

    // The desired color pattern ID (e.g., 21 for GPP) passed from autonomous or configuration
    public final int shootingOrder;

    // The state machine that manages the timing and logic of the shots
    private final ShootingStateMachine stateMachine;

    // Minimum flywheel velocity (ticks per second or RPM) required to safely fire
    private static final double SHOOTER_MIN_VELOCITY = 1000;

    /**
     * Constructor initializes hardware and prepares the state machine.
     * @param hardwareMap The OpMode hardwareMap for device initialization.
     * @param shootingOrder The sequence ID representing the color motif to follow.
     */
    public Transfer(HardwareMap hardwareMap, int shootingOrder) {
        kickers = new TransferServos(hardwareMap);
        shooter = new Flywheel(hardwareMap);
        colorSensors = new TransferColorSensor(hardwareMap);

        this.shootingOrder = shootingOrder;
        this.stateMachine = new ShootingStateMachine(this);
    }

    /**
     * Safety check to ensure the flywheel is spinning fast enough to shoot.
     * Prevents the kicker from jamming a game element into a slow or stationary motor.
     */
    public boolean isShooterAlive() {
        return shooter.getVelocity() > SHOOTER_MIN_VELOCITY;
    }

    /**
     * Actuates a specific kicker servo to the "up" (firing) position.
     * @param kicker The index of the kicker (1, 2, or 3).
     */
    public void kickerUp(int kicker) {
        // Safety: Do not actuate if the shooter isn't at speed
        if (!isShooterAlive()) return;

        switch (kicker) {
            case 1:
                kickers.kicker1GoUp();
                break;
            case 2:
                kickers.kicker2GoUp();
                break;
            case 3:
                kickers.kicker3GoUp();
                break;
        }
    }

    /**
     * Resets all kicker servos to their retracted (down) positions.
     */
    public void lowerAllKickers() {
        kickers.kicker1GoDown();
        kickers.kicker2GoDown();
        kickers.kicker3GoDown();
    }

    /**
     * Triggers a fresh read from the I2C color sensors.
     */
    public void updateColors() {
        colorSensors.update();
    }

    /**
     * Gets the color detected by a specific sensor and converts it to an integer ID.
     * 1 = GREEN, 2 = PURPLE, 0 = NONE.
     * @param sensor The sensor index (1, 2, or 3).
     */
    public int getColor(int sensor) {
        TransferColorSensor.DetectedColor color;

        // Route the request to the specific hardware sensor
        switch (sensor) {
            case 1:
                color = colorSensors.colorOfSensor1();
                break;
            case 2:
                color = colorSensors.colorOfSensor2();
                break;
            case 3:
                color = colorSensors.colorOfSensor3();
                break;
            default:
                return 0;
        }

        // Map the Enum value to an Integer for the State Machine logic
        switch (color) {
            case GREEN:
                return 1;
            case PURPLE:
                return 2;
            case NONE:
            default:
                return 0;
        }
    }

    /**
     * High-level command to execute a shot based on the pre-defined color motif.
     */
    public void shootInOrder() {
        stateMachine.shootInOrder();
    }

    /**
     * High-level command to execute a shot in simple 1-2-3 order.
     */
    public void shootSequential() {
        stateMachine.shootSequential();
    }

    /**
     * Polls the state machine to see if the multi-shot sequence has finished.
     */
    public boolean isShootingComplete() {
        return stateMachine.isShootingComplete();
    }
}
