package org.firstinspires.ftc.teamcode.layeredFinal.physical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Physical layer wrapper for the three kicker servos using a State Machine.
 * This class translates abstract goals (UP/DOWN) into physical PWM signals,
 * providing a reliable way to track kicker positions without constant hardware polling.
 */
public class SmartKickServo {

    /**
     * Enum defining the two possible logical states for any kicker.
     */
    public enum KickerState {
        UP,    // Firing position
        DOWN   // Resting/Loading position
    }

    // Hardware handles for the REV Control/Expansion Hub ports
    private final Servo kickServo1;
    private final Servo kickServo2;
    private final Servo kickServo3;

    /** * Internal state tracking variables.
     * We track the state in software so we can check kicker status instantly
     * without the latency of reading from the I2C/Hardware bus.
     */
    private KickerState state1 = KickerState.DOWN;
    private KickerState state2 = KickerState.DOWN;
    private KickerState state3 = KickerState.DOWN;

    // Calibrated Servo Positions (0.0 to 1.0)
    // These should be adjusted based on physical testing of the linkage
    final double UP_POS = 1.0;
    final double DOWN_POS = 0.0;

    /**
     * Constructor: Initializes servos and forces them into a safe starting position.
     * @param hardwareMap The hardwareMap from the OpMode.
     */
    public SmartKickServo(HardwareMap hardwareMap) {
        // These names must match the configuration on the Driver Station
        kickServo1 = hardwareMap.get(Servo.class, "kick1");
        kickServo2 = hardwareMap.get(Servo.class, "kick2");
        kickServo3 = hardwareMap.get(Servo.class, "kick3");

        // Initialization: Ensure servos are retracted so they don't block intake
        setKick1Down();
        setKick2Down();
        setKick3Down();
    }

    /**
     * Hardware Sync: Forces the physical servos to move to the positions
     * required by the current internal states.
     */
    public void update() {
        kickServo1.setPosition(state1 == KickerState.UP ? UP_POS : DOWN_POS);
        kickServo2.setPosition(state2 == KickerState.UP ? UP_POS : DOWN_POS);
        kickServo3.setPosition(state3 == KickerState.UP ? UP_POS : DOWN_POS);
    }

    // --- State Setters ---
    // These methods update the software state and immediately trigger a hardware update.

    public void setKick1Up() { state1 = KickerState.UP; update(); }
    public void setKick1Down() { state1 = KickerState.DOWN; update(); }

    public void setKick2Up() { state2 = KickerState.UP; update(); }
    public void setKick2Down() { state2 = KickerState.DOWN; update(); }

    public void setKick3Up() { state3 = KickerState.UP; update(); }
    public void setKick3Down() { state3 = KickerState.DOWN; update(); }

    // --- State Checkers ---
    // Used by higher-level logic (like TransferServos) to prevent double-firing.

    /** @return true if Kicker 1 is logically in the UP state. */
    public boolean checkKick1Pos() { return state1 == KickerState.UP; }

    /** @return true if Kicker 2 is logically in the UP state. */
    public boolean checkKick2Pos() { return state2 == KickerState.UP; }

    /** @return true if Kicker 3 is logically in the UP state. */
    public boolean checkKick3Pos() { return state3 == KickerState.UP; }

    /**
     * Physical Verification: Directly queries the last commanded position of the hardware.
     * Useful for debugging or when you need absolute confirmation of the PWM output.
     * @param kickerNumber The ID of the kicker (1-3).
     * @return true if the commanded position is past the halfway threshold.
     */
    public boolean isHardwareUp(int kickerNumber) {
        switch (kickerNumber) {
            case 1: return kickServo1.getPosition() > 0.6;
            case 2: return kickServo2.getPosition() > 0.6;
            case 3: return kickServo3.getPosition() > 0.6;
            default: return false;
        }
    }
}
