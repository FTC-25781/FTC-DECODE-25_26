package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartKickServo;

/**
 * Manages the high-level logic for the three kicker servos.
 * Its primary responsibility is "mutual exclusion"—ensuring that only one
 * kicker is active at a time to prevent jams.
 */
public class TransferServos {
    // Reference to the physical hardware implementation (servo positions/config)
    SmartKickServo servos;

    public TransferServos(HardwareMap hardwareMap) {
        servos = new SmartKickServo(hardwareMap);
    }

    /**
     * Commands Kicker 1 to the firing position.
     * Includes a safety check to prevent mechanical interference.
     */
    public void kicker1GoUp() {
        if (checkIfSafeToGoUp()) {
            servos.setKick1Up();
        }
    }

    /** Sets Kicker 1 back to the resting/intake position. */
    public void kicker1GoDown() { servos.setKick1Down(); }

    /**
     * Commands Kicker 2 to the firing position.
     */
    public void kicker2GoUp() {
        if (checkIfSafeToGoUp()) {
            servos.setKick2Up();
        }
    }

    /** Sets Kicker 2 back to the resting/intake position. */
    public void kicker2GoDown() { servos.setKick2Down(); }

    /**
     * Commands Kicker 3 to the firing position.
     */
    public void kicker3GoUp() {
        if (checkIfSafeToGoUp()) {
            servos.setKick3Up();
        }
    }

    /** Sets Kicker 3 back to the resting/intake position. */
    public void kicker3GoDown() { servos.setKick3Down(); }

    /**
     * Mechanical Guard: Prevents a kicker from firing if any of the three
     * are currently in the 'Up' position.
     * * @return true if all kickers are currently down/safe.
     */
    public boolean checkIfSafeToGoUp() {
        // Logic: (Not Kicker 1 Up) AND (Not Kicker 2 Up) AND (Not Kicker 3 Up)
        return !servos.checkKick1Pos() && !servos.checkKick2Pos() && !servos.checkKick3Pos();
    }
}
