package org.firstinspires.ftc.teamcode.layered.physical1;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoForTransfer {
    private final Servo transferServo;
    private final LimitSwitch limitSwitches;
    private final ElapsedTime debounceTimer = new ElapsedTime(); // for limit switch not going too fast

    private static final double TOP_POS = 0.9;
    private static final double DOWN_POS = 0.1;
    private static final double DEBOUNCE_DELAY = 0.25; // seconds

    private boolean movingUp = false;
    private boolean movingDown = false;

    public ServoForTransfer(HardwareMap hardwareMap) {
        transferServo = hardwareMap.get(Servo.class, "liftServo");
        limitSwitches = new LimitSwitch(hardwareMap);
    }

    public void update() {
        if (debounceTimer.seconds() < DEBOUNCE_DELAY) return;

        if (movingUp && limitSwitches.isTopPressed()) {
            transferServo.setPosition(transferServo.getPosition());
            movingUp = false;
        } else if (movingDown && limitSwitches.isBottomPressed()) {
            transferServo.setPosition(transferServo.getPosition());
            movingDown = false;
        }
    }

    public void moveUp() {
        if(limitSwitches.isTopPressed()) return;
        transferServo.setPosition(TOP_POS);
        movingUp = true;
    }
    public void moveDown() {
        if(limitSwitches.isBottomPressed()) return;
        transferServo.setPosition(DOWN_POS);
        movingDown = true;
    }
}

