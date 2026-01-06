package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.TransferServos;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartLimelight;

@TeleOp(name = "Kickers with color sensor", group = "Test")
public class KickersWithLimelight extends OpMode {
    TransferServos servos;
    SmartLimelight limelight;
    private final ElapsedTime stateTimer = new ElapsedTime();
    int id;

    // Define states for the sequence
    enum State {
        SEARCHING,
        RAISE_FIRST,
        WAIT_FIRST,
        RAISE_SECOND,
        WAIT_SECOND,
        RAISE_THIRD,
        WAIT_THIRD,
        DONE
    }

    State currentState = State.SEARCHING;
    int kickerWithGreen = 1;  // Which kicker has the green sample (always 1)

    @Override
    public void init() {
        servos = new TransferServos(hardwareMap);
        limelight = new SmartLimelight(hardwareMap);
        currentState = State.SEARCHING;
    }

    @Override
    public void loop() {
        telemetry.addData("Current State", currentState);
        telemetry.addData("Timer", "%.2f", stateTimer.seconds());
        telemetry.addData("Tag ID", id);
        telemetry.addData("Green Kicker", kickerWithGreen);

        switch (currentState) {
            case SEARCHING:
                id = limelight.getAprilTagID();

                if (id == 21 || id == 22 || id == 23) {
                    telemetry.addData("Tag Detected!", id);
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
                    servos.kicker1GoUp();  // Green first
                } else {  // id == 22 or 23
                    servos.kicker2GoUp();  // Purple first
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
                    servos.kicker2GoUp();  // Purple second
                } else if (id == 22) {
                    servos.kicker1GoUp();  // Green second
                } else {  // id == 23
                    servos.kicker3GoUp();  // Purple second
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
                    servos.kicker3GoUp();  // Purple third
                } else {  // id == 23
                    servos.kicker1GoUp();  // Green third
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
                telemetry.addData("Status", "Sequence Complete");
                allKickersDown();
                break;
        }

        servos.update();
        telemetry.update();
    }

    private void allKickersDown() {
        servos.kicker1GoDown();
        servos.kicker2GoDown();
        servos.kicker3GoDown();
    }
}
