package org.firstinspires.ftc.teamcode.layeredFinal.physical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Physical layer wrapper for the three kicker servos using a global State Machine.
 * This ensures that only one kicker can be active at a time, matching the
 * "mutual exclusion" logic required by the higher-level transfer layers.
 */
public class SmartKickServo {
    /**
     * Enum representing the mutually exclusive states of the entire kicker system.
     */
    public enum KickerState {
        KICK1_UP,
        KICK2_UP,
        KICK3_UP,
        ALL_DOWN
    }

    private final Servo kickServo1;
    private final Servo kickServo2;
    private final Servo kickServo3;

    // The single source of truth for the entire kicker assembly
    private KickerState currentState = KickerState.ALL_DOWN;

    // Calibrated positions
    final double UP_POS = 0.28;
    final double DOWN_POS = 0.0;

    public SmartKickServo(HardwareMap hardwareMap) {
        kickServo1 = hardwareMap.get(Servo.class, "kick1");
        kickServo2 = hardwareMap.get(Servo.class, "kick2");
        kickServo3 = hardwareMap.get(Servo.class, "kick3");

        kickServo3.setDirection(Servo.Direction.REVERSE);

        // Initialization: Force hardware to match the default ALL_DOWN state
        update();
    }

    /**
     * Hardware Sync: Translates the global state into specific PWM signals for
     * all three servos. This is the only place where setPosition is called.
     */
    public void update() {
        switch (currentState) {
            case KICK1_UP:
                kickServo1.setPosition(UP_POS);
                kickServo2.setPosition(DOWN_POS);
                kickServo3.setPosition(DOWN_POS);
                break;
            case KICK2_UP:
                kickServo1.setPosition(DOWN_POS);
                kickServo2.setPosition(UP_POS);
                kickServo3.setPosition(DOWN_POS);
                break;
            case KICK3_UP:
                kickServo1.setPosition(DOWN_POS);
                kickServo2.setPosition(DOWN_POS);
                kickServo3.setPosition(UP_POS);
                break;
            case ALL_DOWN:
            default:
                kickServo1.setPosition(DOWN_POS);
                kickServo2.setPosition(DOWN_POS);
                kickServo3.setPosition(DOWN_POS);
                break;
        }
    }

    // --- State Setters ---
    // These methods change the goal and immediately push that goal to hardware.
    public void setKick1Up()   { currentState = KickerState.KICK1_UP; update(); }
    public void setKick2Up()   { currentState = KickerState.KICK2_UP; update(); }
    public void setKick3Up()   { currentState = KickerState.KICK3_UP; update(); }
    public void setKick1Down() { currentState = KickerState.ALL_DOWN; update(); }
    public void setKick2Down() { currentState = KickerState.ALL_DOWN; update(); }
    public void setKick3Down() { currentState = KickerState.ALL_DOWN; update(); }

    // --- State Checkers ---
    // Used by higher layers to verify if a kicker is currently deployed.

    /** @return true if the system is currently in the KICK1_UP state. */
    public boolean checkKick1Pos() { return currentState == KickerState.KICK1_UP; }

    /** @return true if the system is currently in the KICK2_UP state. */
    public boolean checkKick2Pos() { return currentState == KickerState.KICK2_UP; }

    /** @return true if the system is currently in the KICK3_UP state. */
    public boolean checkKick3Pos() { return currentState == KickerState.KICK3_UP; }

    /**
     * Helper to get the current state for telemetry or debugging.
     */
    public KickerState getCurrentState() {
        return currentState;
    }
}
