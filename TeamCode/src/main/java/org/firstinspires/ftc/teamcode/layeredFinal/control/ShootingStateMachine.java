package org.firstinspires.ftc.teamcode.layeredFinal.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ShootingStateMachine {

    private final Transfer transfer;

    private enum ShootState {
        IDLE, KICKING, WAITING
    }

    private ShootState state = ShootState.IDLE;

    private int sequenceIndex = 0; // 0,1,2
    private final boolean[] fired = new boolean[3];

    private boolean hasStarted = false;

    private final ElapsedTime timer = new ElapsedTime();

    private static final double KICK_UP_TIME = 0.3;
    private static final double KICK_DOWN_TIME = 0.2;

    public ShootingStateMachine(Transfer transfer) {
        this.transfer = transfer;
    }

    // Shoots based on motif value
    public void shootInOrder() {
        if (!transfer.isShooterAlive()) {
            reset();
            return;
        }

        transfer.updateColors();

        int targetColor = ShootingOrderHelper.getTargetColorForPosition(
                transfer.shootingOrder, sequenceIndex
        );

        int kicker = findMatchingKicker(targetColor);
        process(kicker);
    }

    // If we don't want to go off the value of the motif we can shoot 1->2->3
    public void shootSequential() {
        if (!transfer.isShooterAlive()) {
            reset();
            return;
        }

        int kicker = sequenceIndex + 1;
        process(kicker);
    }

    // Handles the cases of each kicker
    private void process(int kicker) {
        switch (state) {

            case IDLE:
                if (kicker != -1 && kicker <= 3 && !fired[kicker - 1]) {
                    transfer.kickerUp(kicker);
                    fired[kicker - 1] = true;
                    hasStarted = true;

                    timer.reset();
                    state = ShootState.KICKING;
                }
                break;

            case KICKING:
                if (timer.seconds() > KICK_UP_TIME) {
                    transfer.lowerAllKickers();
                    timer.reset();
                    state = ShootState.WAITING;
                }
                break;

            case WAITING:
                if (timer.seconds() > KICK_DOWN_TIME) {
                    sequenceIndex++;

                    if (sequenceIndex >= 3) {
                        reset();
                    } else {
                        state = ShootState.IDLE;
                    }
                }
                break;
        }
    }

    // Allows us to find a kicker that has not already shot yet and has the color we need
    private int findMatchingKicker(int targetColor) {
        for (int i = 1; i <= 3; i++) {
            if (!fired[i - 1] && transfer.getColor(i) == targetColor) {
                return i;
            }
        }
        return -1;
    }

    // Reset the entire system
    private void reset() {
        state = ShootState.IDLE;
        sequenceIndex = 0;
        hasStarted = false;

        for (int i = 0; i < 3; i++) fired[i] = false;

        transfer.lowerAllKickers();
    }

    // Check that we have shot all balls
    public boolean isShootingComplete() {
        return hasStarted && state == ShootState.IDLE && sequenceIndex == 0;
    }
}
