package org.firstinspires.ftc.teamcode.layeredFinal.physical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SmartKickServo {
    Servo kickServo1;
    Servo kickServo2;
    Servo kickServo3;

    final double UP_POS = 1.0; // TODO: GET REAL VAL
    final double DOWN_POS = 0.0; // TODO: GET REAL VAL

    public SmartKickServo(HardwareMap hardwareMap) {
        kickServo1 = hardwareMap.get(Servo.class, "kick1");
        kickServo2 = hardwareMap.get(Servo.class, "kick2");
        kickServo3 = hardwareMap.get(Servo.class, "kick3");
    }

    public void setKick1Up() {
        kickServo1.setPosition(UP_POS);
    }

    public void setKick1Down() {
        kickServo1.setPosition(DOWN_POS);
    }

    public void setKick2Up() {
        kickServo2.setPosition(UP_POS);
    }

    public void setKick2Down() {
        kickServo2.setPosition(DOWN_POS);
    }

    public void setKick3Up() {
        kickServo3.setPosition(UP_POS);
    }

    public void setKick3Down() {
        kickServo3.setPosition(DOWN_POS);
    }
}
