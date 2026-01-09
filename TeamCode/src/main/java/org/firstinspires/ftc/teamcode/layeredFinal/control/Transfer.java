package org.firstinspires.ftc.teamcode.layeredFinal.control;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.layeredFinal.logical.Limelight;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.TransferServos;

public class Transfer {
    // TODO: Adding color sensor after calibrated
    Limelight limelight;
    TransferServos transferServos;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final int kickerWithGreen = 1; // TODO: Set to "0" when color sensor added
    public int id = 21;

    public enum State {
        IDLE,           // Not firing, waiting for command
        SEARCHING,      // Looking for AprilTag
        RAISE_FIRST,
        WAIT_FIRST,
        RAISE_SECOND,
        WAIT_SECOND,
        RAISE_THIRD,
        WAIT_THIRD,
        DONE
    }

    public  State currentState = State.IDLE;
    private boolean sequenceActiveInOrder = false;
    private boolean sequenceActiveRandom = false;

    public Transfer(HardwareMap hardwareMap) {
        limelight = new Limelight(hardwareMap);
        transferServos = new TransferServos(hardwareMap);
    }

    public void startKickSequenceInOrder() {
        if (currentState == State.IDLE || currentState == State.DONE) {
            currentState = State.SEARCHING;
            sequenceActiveInOrder = true;
            id = 0;
        }
    }

    public void startKickSequenceRandomly() {
        if (currentState == State.IDLE || currentState == State.DONE) {
            currentState = State.RAISE_FIRST;
            sequenceActiveRandom = true;
        }
    }

    public void update() {
        transferServos.update();

        // TODO: Add color sensor when calibrated
        // kickerWithGreen = colorSensor.getKickerWithGreen();

        // Run the state machine if a sequence is active
        if (sequenceActiveInOrder) {
            kickWithLimelight(id);
        }

        if (sequenceActiveRandom) {
            kick();
            stateTimer.reset();
        }
    }

    private void kickWithLimelight(int id) {
        switch (currentState) {
            case SEARCHING:
                if (id == 21 || id == 22 || id == 23) {
                    // ID 21 = fire kicker 1 (green), then 2, then 3
                    // ID 22 = fire kicker 2, then 1 (green), then 3
                    // ID 23 = fire kicker 2, then 3, then 1 (green)
                    stateTimer.reset();
                    currentState = State.RAISE_FIRST;
                }
                break;

            case RAISE_FIRST:
                // First kicker to fire depends on ID
                if (id == 21) {
                    transferServos.kicker1GoUp();  // Green first
                } else {  // id == 22 or 23
                    transferServos.kicker2GoUp();  // Purple first
                }
                stateTimer.reset();
                currentState = State.WAIT_FIRST;
                break;

            case WAIT_FIRST:
                if (stateTimer.seconds() >= 2.0) {
                    allKickersDown();
                    stateTimer.reset();
                    currentState = State.RAISE_SECOND;
                }
                break;

            case RAISE_SECOND:
                // Second kicker to fire depends on ID
                if (id == 21) {
                    transferServos.kicker2GoUp();  // Purple second
                } else if (id == 22) {
                    transferServos.kicker1GoUp();  // Green second
                } else {  // id == 23
                    transferServos.kicker3GoUp();  // Purple second
                }
                stateTimer.reset();
                currentState = State.WAIT_SECOND;
                break;

            case WAIT_SECOND:
                if (stateTimer.seconds() >= 2.0) {
                    allKickersDown();
                    stateTimer.reset();
                    currentState = State.RAISE_THIRD;
                }
                break;

            case RAISE_THIRD:
                // Third kicker to fire depends on ID
                if (id == 21 || id == 22) {
                    transferServos.kicker3GoUp();  // Purple third
                } else {  // id == 23
                    transferServos.kicker1GoUp();  // Green third
                }
                stateTimer.reset();
                currentState = State.WAIT_THIRD;
                break;

            case WAIT_THIRD:
                if (stateTimer.seconds() >= 2.0) {
                    allKickersDown();
                    stateTimer.reset();
                    currentState = State.DONE;
                }
                break;

            case DONE:
                allKickersDown();
                sequenceActiveInOrder = false;
                currentState = State.IDLE;
                break;

            case IDLE:
                break;
        }
    }

    private void kick() {
        switch (currentState) {
            case SEARCHING:
                break;

            case RAISE_FIRST:
                // First kicker to fire depends on ID
                transferServos.kicker1GoUp();  // Green first
                stateTimer.reset();
                currentState = State.WAIT_FIRST;
                break;

            case WAIT_FIRST:
                if (stateTimer.seconds() >= 2.0) {
                    allKickersDown();
                    stateTimer.reset();
                    currentState = State.RAISE_SECOND;
                }
                break;

            case RAISE_SECOND:
                transferServos.kicker2GoUp();
                stateTimer.reset();
                currentState = State.WAIT_SECOND;
                break;

            case WAIT_SECOND:
                if (stateTimer.seconds() >= 2.0) {
                    allKickersDown();
                    stateTimer.reset();
                    currentState = State.RAISE_THIRD;
                }
                break;

            case RAISE_THIRD:
                // Third kicker to fire depends on ID
                transferServos.kicker3GoUp();  // Purple third
                stateTimer.reset();
                currentState = State.WAIT_THIRD;
                break;

            case WAIT_THIRD:
                if (stateTimer.seconds() >= 2.0) {
                    allKickersDown();
                    stateTimer.reset();
                    currentState = State.DONE;
                }
                break;

            case DONE:
                allKickersDown();
                sequenceActiveInOrder = false;
                currentState = State.IDLE;
                break;

            case IDLE:
                break;
        }
    }

    private void allKickersDown() {
        transferServos.kicker1GoDown();
        transferServos.kicker2GoDown();
        transferServos.kicker3GoDown();
    }

    public boolean isFiringInOrder() {
        return sequenceActiveInOrder;
    }

    public boolean isFiringRandomly() {
        return sequenceActiveRandom;
    }

    public void reset() {
        currentState = State.IDLE;
        sequenceActiveInOrder = false;
        sequenceActiveRandom = false;
        allKickersDown();
    }
}
