package org.firstinspires.ftc.teamcode.layeredFinal.control;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.layeredFinal.logical.Limelight;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.TransferColorSensor;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.TransferServos;

public class Transfer {
    TransferServos transferServos;
    TransferColorSensor transferColors;

    private final ElapsedTime stateTimer = new ElapsedTime();
    public int kickerWithGreen = 1;
    public int id = 21;
    private int purpleKickerCount = 0;

    public double kicker1_time_value = 0.14;
    public double kicker1_servo_lower_time = 0.02;
    public double kicker2_time_value = 0.18;
    public double kicker2_servo_lower_time = 0.065;
    public double kicker3_time_value = 0.14;
    public double kicker3_servo_lower_time = 0;

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

    public State currentState = State.IDLE;
    private boolean sequenceActiveInOrder = false;
    private boolean sequenceActiveRandom = false;

    public Transfer(HardwareMap hardwareMap) {
        transferServos = new TransferServos(hardwareMap);
        transferColors = new TransferColorSensor(hardwareMap);
    }

    public void startKickSequenceInOrder(int aprilTagId) {
        if (currentState == State.IDLE || currentState == State.DONE) {
            currentState = State.SEARCHING;
            sequenceActiveInOrder = true;
            id = aprilTagId;
            purpleKickerCount = 0;
        }
    }

    public void startKickSequenceRandomly() {
        if (currentState == State.IDLE || currentState == State.DONE) {
            currentState = State.RAISE_FIRST;
            sequenceActiveRandom = true;
        }
    }

    public void update() {
        if (transferColors.colorOfSensor1() == TransferColorSensor.DetectedColor.GREEN) {
            kickerWithGreen = 1;
        } else if (transferColors.colorOfSensor2() == TransferColorSensor.DetectedColor.GREEN) {
            kickerWithGreen = 2;
        } else if (transferColors.colorOfSensor3() == TransferColorSensor.DetectedColor.GREEN) {
            kickerWithGreen = 3;
        }

        transferServos.update();
        transferColors.update();

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
                if (stateTimer.seconds() >= kicker1_time_value) {
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
                if (stateTimer.seconds() >= kicker1_servo_lower_time) {
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
                if (stateTimer.seconds() >= kicker2_time_value) {
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
                if (stateTimer.seconds() >= kicker2_servo_lower_time) {
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
                if (stateTimer.seconds() >= kicker3_time_value) {
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
                if (stateTimer.seconds() >= kicker3_servo_lower_time) {
                    stateTimer.reset();
                    currentState = State.DONE;
                }
                break;

            case DONE:
                allKickersDown();
                sequenceActiveInOrder = false;
                currentState = State.DONE;
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
                transferServos.kicker1GoUp();
                stateTimer.reset();
                currentState = State.WAIT_FIRST;
                break;

            case WAIT_FIRST:
                if (stateTimer.seconds() >= kicker1_time_value) {
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
                if (stateTimer.seconds() >= kicker1_servo_lower_time) {
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
                if (stateTimer.seconds() >= kicker2_time_value) {
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
                if (stateTimer.seconds() >= kicker2_servo_lower_time) {
                    stateTimer.reset();
                    currentState = State.RAISE_THIRD;
                }
                break;

            case RAISE_THIRD:
                transferServos.kicker3GoUp();
                stateTimer.reset();
                currentState = State.WAIT_THIRD;
                break;

            case WAIT_THIRD:
                if (stateTimer.seconds() >= kicker3_time_value) {
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
                if (stateTimer.seconds() >= kicker3_servo_lower_time) {
                    stateTimer.reset();
                    currentState = State.DONE;
                }
                break;

            case DONE:
                allKickersDown();
                sequenceActiveRandom = false;
                currentState = State.DONE;
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
        // Raise a purple kicker (not the green one)
        // Tracks count to raise different purple kickers on subsequent calls
        if (kickerWithGreen == 1) {
            if (purpleKickerCount == 0) {
                transferServos.kicker2GoUp();
                purpleKickerCount++;
            } else {
                transferServos.kicker3GoUp();
            }
        } else if (kickerWithGreen == 2) {
            if (purpleKickerCount == 0) {
                transferServos.kicker1GoUp();
                purpleKickerCount++;
            } else {
                transferServos.kicker3GoUp();
            }
        } else {  // kickerWithGreen == 3
            if (purpleKickerCount == 0) {
                transferServos.kicker1GoUp();
                purpleKickerCount++;
            } else {
                transferServos.kicker2GoUp();
            }
        }
    }

    private void allKickersDown() {
        transferServos.kicker1GoDown();
        transferServos.kicker2GoDown();
        transferServos.kicker3GoDown();
    }

    public void reset() {
        currentState = State.IDLE;
        sequenceActiveInOrder = false;
        sequenceActiveRandom = false;
        purpleKickerCount = 0;
        allKickersDown();
    }
}
