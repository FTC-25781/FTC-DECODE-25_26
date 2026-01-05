package org.firstinspires.ftc.teamcode.layeredFinal.control;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Flywheel;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Limelight;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.TransferColorSensor;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.TransferServos;

/**
 * The Transfer class coordinates hardware components and orchestrates the shooting sequence.
 * It integrates vision data from the database to determine the specific shooting order
 * required for the current match scenario.
 */
public class Transfer {
    // Hardware abstractions
    private final TransferServos kickers;
    private final Flywheel shooter;
    private final TransferColorSensor colorSensors;
    private Limelight limelight;

    /** * The desired color pattern ID retrieved from the database.
     * This is set once during initialization and remains constant for the OpMode.
     */
    public final int shootingOrder;

    // Logic controller for the firing sequence
    private final ShootingStateMachine stateMachine;

    // Minimum velocity (ticks/sec) required before the kickers are allowed to fire
    private static final double SHOOTER_MIN_VELOCITY = 1000;

    /**
     * Initializes the transfer system and pulls configuration data from the Limelight database.
     * @param hardwareMap The OpMode hardware map for device initialization.
     */
    public Transfer(HardwareMap hardwareMap) {
        // Initialize hardware layers
        this.kickers = new TransferServos(hardwareMap);
        this.shooter = new Flywheel(hardwareMap);
        this.colorSensors = new TransferColorSensor(hardwareMap);
        this.limelight = new Limelight(hardwareMap);

        /*
         * DATABASE INTEGRATION:
         * We query the Limelight logical wrapper to see what the last detected AprilTag was.
         * This allows TeleOp to "inherit" knowledge from the Autonomous phase.
         */
        int lastSeenTag = limelight.getLastLoggedID();

        // Default to 0 (or a safe sequence) if no tag has been logged yet
        this.shootingOrder = (lastSeenTag != -1) ? lastSeenTag : 0;

        // Initialize the state machine with this transfer instance
        this.stateMachine = new ShootingStateMachine(this);
    }

    /**
     * Safety check: returns true if the flywheel is spinning fast enough to shoot.
     */
    public boolean isShooterAlive() {
        return shooter.getVelocity() > SHOOTER_MIN_VELOCITY;
    }

    /**
     * Actuates a specific kicker to the firing position if the shooter is at speed.
     * @param kicker The index (1-3) of the servo to actuate.
     */
    public void kickerUp(int kicker) {
        // Safety interlock to prevent jams
        if (!isShooterAlive()) return;

        switch (kicker) {
            case 1: kickers.kicker1GoUp(); break;
            case 2: kickers.kicker2GoUp(); break;
            case 3: kickers.kicker3GoUp(); break;
        }
    }

    /**
     * Resets all kicker servos to their resting (retracted) positions.
     */
    public void lowerAllKickers() {
        kickers.kicker1GoDown();
        kickers.kicker2GoDown();
        kickers.kicker3GoDown();
    }

    /**
     * Polls the I2C color sensors for new data.
     */
    public void update() {
        colorSensors.update();
        kickers.update();
    }

    /**
     * Gets the current color state of a specific sensor.
     * @param sensor The index (1-3) of the sensor to check.
     * @return 1 for GREEN, 2 for PURPLE, 0 for NONE/OTHER.
     */
    public int getColor(int sensor) {
        TransferColorSensor.DetectedColor color;
        switch (sensor) {
            case 1: color = colorSensors.colorOfSensor1(); break;
            case 2: color = colorSensors.colorOfSensor2(); break;
            case 3: color = colorSensors.colorOfSensor3(); break;
            default: return 0;
        }

        // Map sensor enums to integer IDs for state machine logic
        switch (color) {
            case GREEN: return 1;
            case PURPLE: return 2;
            default: return 0;
        }
    }

    /**
     * Starts the automated shooting sequence using the database-provided shootingOrder.
     */
    public void shootInOrder() {
        stateMachine.shootInOrder();
    }

    /**
     * Starts a standard sequential shot (1, then 2, then 3).
     */
    public void shootSequential() {
        stateMachine.shootSequential();
    }

    /**
     * Checks if the state machine has finished the current shooting routine.
     */
    public boolean isShootingComplete() {
        return stateMachine.isShootingComplete();
    }
}
