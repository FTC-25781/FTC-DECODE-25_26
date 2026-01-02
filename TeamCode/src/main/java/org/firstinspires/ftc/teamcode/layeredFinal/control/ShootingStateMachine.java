package org.firstinspires.ftc.teamcode.layeredFinal.control;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Manages the timing and logic for firing three kickers in a specific order.
 * This ensures rings/balls are fired only when the shooter is ready and in the correct sequence.
 */
public class ShootingStateMachine {

    private final Transfer transfer;

    // Internal states for the shooting cycle
    private enum ShootState {
        IDLE,       // Waiting for a command to shoot
        KICKING,    // Kicker is currently extended/up
        WAITING     // Kicker is retracting/down, waiting for mechanical settle
    }

    private ShootState state = ShootState.IDLE;

    private int sequenceIndex = 0;           // Tracks which shot (1st, 2nd, or 3rd) we are on
    private final boolean[] fired = new boolean[3]; // Tracks which physical kickers have been used

    private boolean hasStarted = false;      // Flags if a shooting sequence is currently active

    private final ElapsedTime timer = new ElapsedTime();

    // Timing constants (in seconds) for the mechanical motion of the kickers
    private static final double KICK_UP_TIME = 0.3;
    private static final double KICK_DOWN_TIME = 0.2;

    public ShootingStateMachine(Transfer transfer) {
        this.transfer = transfer;
    }

    /**
     * Attempts to shoot based on a specific color pattern (e.g., Red-Blue-White).
     * It finds which physical kicker currently holds the required color for the current sequence.
     */
    public void shootInOrder() {
        // Safety check: Don't fire if the shooter motor isn't running/at speed
        if (!transfer.isShooterAlive()) {
            reset();
            return;
        }

        transfer.updateColors(); // Refresh sensor data

        // Determine which color we need next based on the motif/pattern
        int targetColor = ShootingOrderHelper.getTargetColorForPosition(
                transfer.shootingOrder, sequenceIndex
        );

        // Map that required color to a physical kicker (1, 2, or 3)
        int kicker = findMatchingKicker(targetColor);
        process(kicker);
    }

    /**
     * Shoots kickers in simple numerical order (1, then 2, then 3)
     * regardless of the color of the objects they hold.
     */
    public void shootSequential() {
        if (!transfer.isShooterAlive()) {
            reset();
            return;
        }

        int kicker = sequenceIndex + 1; // Maps 0,1,2 to kicker IDs 1,2,3
        process(kicker);
    }

    /**
     * The core State Machine logic that handles the timing of the kicker hardware.
     */
    private void process(int kicker) {
        switch (state) {

            case IDLE:
                // Only proceed if a valid kicker is found and it hasn't fired yet
                if (kicker != -1 && kicker <= 3 && !fired[kicker - 1]) {
                    transfer.kickerUp(kicker);    // Actuate hardware
                    fired[kicker - 1] = true;     // Mark as used
                    hasStarted = true;

                    timer.reset();
                    state = ShootState.KICKING;
                }
                break;

            case KICKING:
                // Wait for the kicker to fully extend before retracting
                if (timer.seconds() > KICK_UP_TIME) {
                    transfer.lowerAllKickers();
                    timer.reset();
                    state = ShootState.WAITING;
                }
                break;

            case WAITING:
                // Wait for the kicker to fully retract before moving to the next ring
                if (timer.seconds() > KICK_DOWN_TIME) {
                    sequenceIndex++;

                    // If we've finished all 3 shots, reset the system
                    if (sequenceIndex >= 3) {
                        reset();
                    } else {
                        state = ShootState.IDLE; // Ready for the next kicker in the sequence
                    }
                }
                break;
        }
    }

    /**
     * Scans the 3 kicker slots to find which one contains the color we need.
     * @return The kicker ID (1-3) or -1 if no match is found.
     */
    private int findMatchingKicker(int targetColor) {
        for (int i = 1; i <= 3; i++) {
            // Check if kicker is unused and sensor matches requested color
            if (!fired[i - 1] && transfer.getColor(i) == targetColor) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Resets the state machine variables and hardware to the default state.
     */
    private void reset() {
        state = ShootState.IDLE;
        sequenceIndex = 0;
        hasStarted = false;

        for (int i = 0; i < 3; i++) fired[i] = false;

        transfer.lowerAllKickers();
    }

    /**
     * Used by TeleOp or Autonomous to determine if the shooting sequence is finished.
     */
    public boolean isShootingComplete() {
        return hasStarted && state == ShootState.IDLE && sequenceIndex == 0;
    }
}
