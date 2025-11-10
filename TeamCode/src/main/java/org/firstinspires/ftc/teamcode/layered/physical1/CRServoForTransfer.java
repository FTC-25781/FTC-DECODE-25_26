package org.firstinspires.ftc.teamcode.layered.physical1;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CRServoForTransfer {
    private final CRServo transferServo;
    private final LimitSwitch limitSwitches;
    private final ElapsedTime debounceTimer = new ElapsedTime(); // for limit switch not going too fast

    private static final double POWER_UP = 1.0;
    private static final double POWER_DOWN = -1.0;
    private static final double DEBOUNCE_DELAY = 0.25; // seconds

    private boolean movingUp = false;
    private boolean movingDown = false;

    public CRServoForTransfer(HardwareMap hardwareMap) {
        transferServo = hardwareMap.get(CRServo.class, "liftServo");
        limitSwitches = new LimitSwitch(hardwareMap);
    }

    public void update() {
        if (debounceTimer.seconds() < DEBOUNCE_DELAY) return;

        if (movingUp && limitSwitches.isTopPressed()) {
            transferServo.setPower(0);
            movingUp = false;
        } else if (movingDown && limitSwitches.isBottomPressed()) {
            transferServo.setPower(0);
            movingDown = false;
        }
    }

    public void moveUp() {
        if(limitSwitches.isTopPressed()) return;
        transferServo.setPower(POWER_UP);
        movingUp = true;
    }
    public void moveDown() {
        if(limitSwitches.isBottomPressed()) return;
        transferServo.setPower(POWER_DOWN);
        movingDown = true;
    }
    public boolean isMovingUp() {
        return movingUp;
    }
}

