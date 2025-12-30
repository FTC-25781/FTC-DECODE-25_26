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

    public void shootSequential() {
        if (!transfer.isShooterAlive()) {
            reset();
            return;
        }

        int kicker = sequenceIndex + 1;
        process(kicker);
    }

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

    private int findMatchingKicker(int targetColor) {
        for (int i = 1; i <= 3; i++) {
            if (!fired[i - 1] && transfer.getColor(i) == targetColor) {
                return i;
            }
        }
        return -1;
    }

    private void reset() {
        state = ShootState.IDLE;
        sequenceIndex = 0;
        hasStarted = false;

        for (int i = 0; i < 3; i++) fired[i] = false;

        transfer.lowerAllKickers();
    }

    public boolean isShootingComplete() {
        return hasStarted && state == ShootState.IDLE && sequenceIndex == 0;
    }
}
