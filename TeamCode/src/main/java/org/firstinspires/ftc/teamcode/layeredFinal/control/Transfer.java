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
    public int kickerWithGreen = 1;
    public int id = 21;

    public final double TIME_VALUE = 0.5;
    public final double SERVO_LOWER_TIME = 0.5; // Time for servos to fully lower

    public enum State {
        IDLE,           // Not firing, waiting for command
        SEARCHING,      // Looking for AprilTag
        RAISE_FIRST,
        WAIT_FIRST,
        LOWER_FIRST,
        WAIT_LOWER_FIRST,
        RAISE_SECOND,
        WAIT_SECOND,
        LOWER_SECOND,
        WAIT_LOWER_SECOND,
        RAISE_THIRD,
        WAIT_THIRD,
        LOWER_THIRD,
        WAIT_LOWER_THIRD,
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

        // Run the state machine if a sequence is active
        if (sequenceActiveInOrder) {
            kickWithLimelight(id);
        }

        if (sequenceActiveRandom) {
            kick();
        }
    }

    private void kickWithLimelight(int id) {
        switch (currentState) {
            case SEARCHING:
                if (id == 21 || id == 22 || id == 23) {
                    // ID 21 = fire green kicker first, then others
                    // ID 22 = fire purple kicker, then green, then purple
                    // ID 23 = fire purple kicker, then purple, then green
                    stateTimer.reset();
                    currentState = State.RAISE_FIRST;
                }
                break;

            case RAISE_FIRST:
                // First kicker to fire depends on ID
                if (id == 21) {
                    raiseKicker(kickerWithGreen);  // Green first
                } else {  // id == 22 or 23
                    raisePurpleKicker();  // Purple first
                }
                stateTimer.reset();
                currentState = State.WAIT_FIRST;
                break;

            case WAIT_FIRST:
                if (stateTimer.seconds() >= TIME_VALUE) {
                    stateTimer.reset();
                    currentState = State.LOWER_FIRST;
                }
                break;

            case LOWER_FIRST:
                allKickersDown();
                stateTimer.reset();
                currentState = State.WAIT_LOWER_FIRST;
                break;

            case WAIT_LOWER_FIRST:
                if (stateTimer.seconds() >= SERVO_LOWER_TIME) {
                    stateTimer.reset();
                    currentState = State.RAISE_SECOND;
                }
                break;

            case RAISE_SECOND:
                // Second kicker to fire depends on ID
                if (id == 21) {
                    raisePurpleKicker();  // Purple second
                } else if (id == 22) {
                    raiseKicker(kickerWithGreen);  // Green second
                } else {  // id == 23
                    raisePurpleKicker();  // Purple second (different one)
                }
                stateTimer.reset();
                currentState = State.WAIT_SECOND;
                break;

            case WAIT_SECOND:
                if (stateTimer.seconds() >= TIME_VALUE) {
                    stateTimer.reset();
                    currentState = State.LOWER_SECOND;
                }
                break;

            case LOWER_SECOND:
                allKickersDown();
                stateTimer.reset();
                currentState = State.WAIT_LOWER_SECOND;
                break;

            case WAIT_LOWER_SECOND:
                if (stateTimer.seconds() >= SERVO_LOWER_TIME) {
                    stateTimer.reset();
                    currentState = State.RAISE_THIRD;
                }
                break;

            case RAISE_THIRD:
                // Third kicker to fire depends on ID
                if (id == 21 || id == 22) {
                    raisePurpleKicker();  // Purple third
                } else {  // id == 23
                    raiseKicker(kickerWithGreen);  // Green third
                }
                stateTimer.reset();
                currentState = State.WAIT_THIRD;
                break;

            case WAIT_THIRD:
                if (stateTimer.seconds() >= TIME_VALUE) {
                    stateTimer.reset();
                    currentState = State.LOWER_THIRD;
                }
                break;

            case LOWER_THIRD:
                allKickersDown();
                stateTimer.reset();
                currentState = State.WAIT_LOWER_THIRD;
                break;

            case WAIT_LOWER_THIRD:
                if (stateTimer.seconds() >= SERVO_LOWER_TIME) {
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
                transferServos.kicker1GoUp();  // Green first
                stateTimer.reset();
                currentState = State.WAIT_FIRST;
                break;

            case WAIT_FIRST:
                if (stateTimer.seconds() >= TIME_VALUE) {
                    stateTimer.reset();
                    currentState = State.LOWER_FIRST;
                }
                break;

            case LOWER_FIRST:
                allKickersDown();
                stateTimer.reset();
                currentState = State.WAIT_LOWER_FIRST;
                break;

            case WAIT_LOWER_FIRST:
                if (stateTimer.seconds() >= SERVO_LOWER_TIME) {
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
                if (stateTimer.seconds() >= TIME_VALUE) {
                    stateTimer.reset();
                    currentState = State.LOWER_SECOND;
                }
                break;

            case LOWER_SECOND:
                allKickersDown();
                stateTimer.reset();
                currentState = State.WAIT_LOWER_SECOND;
                break;

            case WAIT_LOWER_SECOND:
                if (stateTimer.seconds() >= SERVO_LOWER_TIME) {
                    stateTimer.reset();
                    currentState = State.RAISE_THIRD;
                }
                break;

            case RAISE_THIRD:
                transferServos.kicker3GoUp();  // Purple third
                stateTimer.reset();
                currentState = State.WAIT_THIRD;
                break;

            case WAIT_THIRD:
                if (stateTimer.seconds() >= TIME_VALUE) {
                    stateTimer.reset();
                    currentState = State.LOWER_THIRD;
                }
                break;

            case LOWER_THIRD:
                allKickersDown();
                stateTimer.reset();
                currentState = State.WAIT_LOWER_THIRD;
                break;

            case WAIT_LOWER_THIRD:
                if (stateTimer.seconds() >= SERVO_LOWER_TIME) {
                    stateTimer.reset();
                    currentState = State.DONE;
                }
                break;

            case DONE:
                allKickersDown();
                sequenceActiveRandom = false;
                currentState = State.IDLE;
                break;

            case IDLE:
                break;
        }
    }

    private void raiseKicker(int kickerNumber) {
        switch (kickerNumber) {
            case 1:
                transferServos.kicker1GoUp();
                break;
            case 2:
                transferServos.kicker2GoUp();
                break;
            case 3:
                transferServos.kicker3GoUp();
                break;
        }
    }

    private void raisePurpleKicker() {
        // Raise the first available purple kicker (not the green one)
        if (kickerWithGreen == 1) {
            transferServos.kicker2GoUp();
        } else if (kickerWithGreen == 2) {
            transferServos.kicker1GoUp();
        } else {
            transferServos.kicker1GoUp();
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
