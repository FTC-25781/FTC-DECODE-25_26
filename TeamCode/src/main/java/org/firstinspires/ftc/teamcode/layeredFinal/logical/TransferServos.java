package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartKickServo;

public class TransferServos {
    SmartKickServo servos;

    public TransferServos(HardwareMap hardwareMap) {
        servos = new SmartKickServo(hardwareMap);
    }

    public void kicker1GoUp() {
        if (checkIfSafeToGoUp()) {
            servos.setKick1Up();
        }
    }
    public void kicker1GoDown() { servos.setKick1Down(); }

    public void kicker2GoUp() {
        if (checkIfSafeToGoUp()) {
            servos.setKick2Up();
        }
    }

    public void kicker2GoDown() { servos.setKick2Down(); }

    public void kicker3GoUp() {
        if (checkIfSafeToGoUp()) {
            servos.setKick3Up();
        }
    }

    public void kicker3GoDown() { servos.setKick3Down(); }

    public boolean checkIfSafeToGoUp() {
        return !servos.checkKick1Pos() && !servos.checkKick2Pos() && !servos.checkKick3Pos();
    }
}
