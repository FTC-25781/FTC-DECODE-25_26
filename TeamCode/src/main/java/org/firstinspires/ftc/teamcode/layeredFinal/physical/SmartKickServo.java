package org.firstinspires.ftc.teamcode.layeredFinal.physical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Physical layer wrapper for the three kicker servos.
 * This class handles the specific servo positions and provides hardware-level
 * status checks for the transfer system.
 */
public class SmartKickServo {
    // Hardware objects for the physical servos
    private final Servo kickServo1;
    private final Servo kickServo2;
    private final Servo kickServo3;

    // Servo position constants (Range 0.0 to 1.0)
    // These represent the physical travel limits of your kickers
    final double UP_POS = 1.0;   // Position to push the ring into the flywheel (TODO: Calibrate)
    final double DOWN_POS = 0.0; // Home position to allow next ring to load (TODO: Calibrate)

    /**
     * Maps the software objects to the Control Hub configuration.
     * @param hardwareMap The OpMode hardwareMap.
     */
    public SmartKickServo(HardwareMap hardwareMap) {
        // These names ("kick1", "kick2", "kick3") must match the Driver Station config
        kickServo1 = hardwareMap.get(Servo.class, "kick1");
        kickServo2 = hardwareMap.get(Servo.class, "kick2");
        kickServo3 = hardwareMap.get(Servo.class, "kick3");
    }

    // --- Kicker 1 Control Functions ---
    public void setKick1Up() {
        kickServo1.setPosition(UP_POS);
    }
    public void setKick1Down() {
        kickServo1.setPosition(DOWN_POS);
    }

    // --- Kicker 2 Control Functions ---
    public void setKick2Up() {
        kickServo2.setPosition(UP_POS);
    }
    public void setKick2Down() {
        kickServo2.setPosition(DOWN_POS);
    }

    // --- Kicker 3 Control Functions ---
    public void setKick3Up() {
        kickServo3.setPosition(UP_POS);
    }
    public void setKick3Down() {
        kickServo3.setPosition(DOWN_POS);
    }

    /**
     * Checks if Kicker 1 is currently in the "Firing" (Up) zone.
     * Uses a threshold (0.6) instead of an exact match (1.0) to account for
     * signal noise or incomplete travel.
     * @return true if the kicker is considered "Up".
     */
    public boolean checkKick1Pos() {
        return kickServo1.getPosition() > 0.6;
    }

    /**
     * Checks if Kicker 2 is currently in the "Firing" (Up) zone.
     * @return true if the kicker is considered "Up".
     */
    public boolean checkKick2Pos() {
        return kickServo2.getPosition() > 0.6;
    }

    /**
     * Checks if Kicker 3 is currently in the "Firing" (Up) zone.
     * @return true if the kicker is considered "Up".
     */
    public boolean checkKick3Pos() {
        return kickServo3.getPosition() > 0.6;
    }
}
