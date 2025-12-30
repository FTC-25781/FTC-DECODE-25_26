package org.firstinspires.ftc.teamcode.layeredFinal.control;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.layeredFinal.logical.Flywheel;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.TransferColorSensor;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartKickServo;

public class Transfer {
    SmartKickServo kickers;
    Flywheel shooter;
    TransferColorSensor colorSensors;
    int shootingOrder;

    // State machine for shooting
    private enum ShootState {
        IDLE, KICKING, WAITING
    }
    private ShootState state = ShootState.IDLE;
    private int currentKicker = 0;
    private ElapsedTime kickTimer = new ElapsedTime();
    private static final double KICK_UP_TIME = 0.3; // Time servo stays up
    private static final double KICK_DOWN_TIME = 0.2; // Time to wait after going down

    public Transfer(HardwareMap hardwareMap, int shootingOrder) {
        kickers = new SmartKickServo(hardwareMap);
        colorSensors = new TransferColorSensor(hardwareMap);
        this.shootingOrder = shootingOrder; // 21 (GPP), 22 (PGP), or 23 (PPG)
    }

    public boolean isShooterAlive() { return shooter.getVelocity() > 1000; }

    public void kicker1Up() { if (isShooterAlive()) kickers.setKick1Up(); }
    public void kicker1Down() { kickers.setKick1Down(); }

    public void kicker2Up() { if (isShooterAlive()) kickers.setKick2Up(); }
    public void kicker2Down() { kickers.setKick2Down(); }

    public void kicker3Up() { if (isShooterAlive()) kickers.setKick3Up(); }
    public void kicker3Down() { kickers.setKick3Down(); }

    public void shootInOrder() {
        if (!isShooterAlive()) {
            resetShootingSequence();
            return;
        }

        colorSensors.update();

        // Determine which kicker to fire based on shooting order
        int targetColor = getTargetColorForPosition(currentKicker);
        int kickerToFire = findKickerWithColor(targetColor);

        switch (state) {
            case IDLE:
                if (kickerToFire != -1) {
                    fireKicker(kickerToFire);
                    state = ShootState.KICKING;
                    kickTimer.reset();
                }
                break;

            case KICKING:
                if (kickTimer.seconds() > KICK_UP_TIME) {
                    // Put the kicker back down
                    lowerAllKickers();
                    state = ShootState.WAITING;
                    kickTimer.reset();
                }
                break;

            case WAITING:
                if (kickTimer.seconds() > KICK_DOWN_TIME) {
                    currentKicker++;
                    if (currentKicker >= 3) {
                        resetShootingSequence();
                    } else {
                        state = ShootState.IDLE;
                    }
                }
                break;
        }
    }

    public void shootSequential() {
        if (!isShooterAlive()) {
            resetShootingSequence();
            return;
        }

        switch (state) {
            case IDLE:
                // Fire kicker based on currentKicker (0=kicker1, 1=kicker2, 2=kicker3)
                fireKicker(currentKicker + 1);
                state = ShootState.KICKING;
                kickTimer.reset();
                break;

            case KICKING:
                if (kickTimer.seconds() > KICK_UP_TIME) {
                    // Put the kicker back down
                    lowerAllKickers();
                    state = ShootState.WAITING;
                    kickTimer.reset();
                }
                break;

            case WAITING:
                if (kickTimer.seconds() > KICK_DOWN_TIME) {
                    currentKicker++;
                    if (currentKicker >= 3) {
                        resetShootingSequence();
                    } else {
                        state = ShootState.IDLE;
                    }
                }
                break;
        }
    }

    private int getTargetColorForPosition(int position) {
        // shootingOrder: 21 = GPP, 22 = PGP, 23 = PPG
        // position: 0, 1, 2 (first, second, third to shoot)
        // color: 1 = green, 2 = purple

        if (shootingOrder == 21) { // GPP
            return (position == 0) ? 1 : 2;
        } else if (shootingOrder == 22) { // PGP
            return (position == 1) ? 1 : 2;
        } else if (shootingOrder == 23) { // PPG
            return (position == 2) ? 1 : 2;
        }
        return 0; // Invalid
    }

    private int findKickerWithColor(int targetColor) {
        // Returns kicker number (1, 2, or 3) or -1 if not found
        if (colorSensors.colorOfSensor1() == targetColor) return 1;
        if (colorSensors.colorOfSensor2() == targetColor) return 2;
        if (colorSensors.colorOfSensor3() == targetColor) return 3;
        return -1; // Color not found
    }

    private void fireKicker(int kickerNum) {
        switch (kickerNum) {
            case 1: kicker1Up(); break;
            case 2: kicker2Up(); break;
            case 3: kicker3Up(); break;
        }
    }

    private void lowerAllKickers() {
        kicker1Down();
        kicker2Down();
        kicker3Down();
    }

    private void resetShootingSequence() {
        state = ShootState.IDLE;
        currentKicker = 0;
        lowerAllKickers();
    }

    public boolean isShootingComplete() {
        return state == ShootState.IDLE && currentKicker == 0;
    }
}
