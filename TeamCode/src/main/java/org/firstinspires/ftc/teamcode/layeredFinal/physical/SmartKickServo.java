package org.firstinspires.ftc.teamcode.layeredFinal.physical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SmartKickServo {
    public enum KickerState {
        KICK1_UP,
        KICK2_UP,
        KICK3_UP,
        ALL_DOWN
    }

    private final Servo kickServo1;
    private final Servo kickServo2;
    private final Servo kickServo3;

    private KickerState currentState = KickerState.ALL_DOWN;

    // Calibrated positions
    final double UP_POS = 0.28;
    final double DOWN_POS = 0.0;

    public SmartKickServo(HardwareMap hardwareMap) {
        kickServo1 = hardwareMap.get(Servo.class, "kick1");
        kickServo2 = hardwareMap.get(Servo.class, "kick2");
        kickServo3 = hardwareMap.get(Servo.class, "kick3");

        kickServo3.setDirection(Servo.Direction.REVERSE);

        update();
    }

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

    public void setKick1Up()   { currentState = KickerState.KICK1_UP; update(); }
    public void setKick2Up()   { currentState = KickerState.KICK2_UP; update(); }
    public void setKick3Up()   { currentState = KickerState.KICK3_UP; update(); }
    public void setKick1Down() { currentState = KickerState.ALL_DOWN; update(); }
    public void setKick2Down() { currentState = KickerState.ALL_DOWN; update(); }
    public void setKick3Down() { currentState = KickerState.ALL_DOWN; update(); }

    public boolean checkKick1Pos() { return currentState == KickerState.KICK1_UP; }
    public boolean checkKick2Pos() { return currentState == KickerState.KICK2_UP; }
    public boolean checkKick3Pos() { return currentState == KickerState.KICK3_UP; }
}
