package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.layeredFinal.logical.TransferColorSensor;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.TransferServos;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartLimelight;

@TeleOp(name = "Kickers with color sensor", group = "Test")
public class KickersWithColorSensor extends OpMode {
    TransferServos servos;
    SmartLimelight limelight;
    TransferColorSensor color; // TODO: Fix color sensor

    private final ElapsedTime stateTimer = new ElapsedTime();

    // Define states for the sequence
    enum State {
        SEARCHING,
        FIRING_GREEN,
        FIRING_OTHERS,
        DONE
    }

    State currentState = State.SEARCHING;
    int kickerWithGreen = 0;

    @Override
    public void init() {
        servos = new TransferServos(hardwareMap);
        limelight = new SmartLimelight(hardwareMap);
        color = new TransferColorSensor(hardwareMap);

        currentState = State.SEARCHING;
    }

    @Override
    public void loop() {
        // Essential: Update sensors every loop
        color.update();

        switch (currentState) {
            case SEARCHING:
                if (limelight.getAprilTagID() == 21) {
                    if (color.colorOfSensor1() == TransferColorSensor.DetectedColor.GREEN) {
                        kickerWithGreen = 1;
                        telemetry.addData("tag", limelight.getAprilTagID());
                        fireGreen();
                    } else if (color.colorOfSensor2() == TransferColorSensor.DetectedColor.GREEN) {
                        kickerWithGreen = 2;
                        telemetry.addData("tag", limelight.getAprilTagID());
                        fireGreen();
                    } else if (color.colorOfSensor3() == TransferColorSensor.DetectedColor.GREEN) {
                        kickerWithGreen = 3;
                        telemetry.addData("tag", limelight.getAprilTagID());
                        fireGreen();
                    }
                }
                break;

            case FIRING_GREEN:
                // Wait for 2 seconds while the kicker is up
                if (stateTimer.seconds() >= 2.0) {
                    allKickersDown();
                    stateTimer.reset();
                    currentState = State.FIRING_OTHERS;
                }
                break;

            case FIRING_OTHERS:
                // Fire the two that WEREN'T green
                if (kickerWithGreen == 1) { servos.kicker2GoUp(); servos.kicker3GoUp(); }
                if (kickerWithGreen == 2) { servos.kicker1GoUp(); servos.kicker3GoUp(); }
                if (kickerWithGreen == 3) { servos.kicker1GoUp(); servos.kicker2GoUp(); }

                if (stateTimer.seconds() >= 2.0) {
                    allKickersDown();
                    currentState = State.DONE;
                }
                break;

            case DONE:
                // Sequence finished. Add a reset condition here if needed (e.g., a button press)
                telemetry.addData("Status", "Sequence Complete");
                break;
        }

        servos.update(); // Apply servo positions
    }

    // Helper methods to keep loop clean
    private void fireGreen() {
        if (kickerWithGreen == 1) servos.kicker1GoUp();
        if (kickerWithGreen == 2) servos.kicker2GoUp();
        if (kickerWithGreen == 3) servos.kicker3GoUp();

        stateTimer.reset();
        currentState = State.FIRING_GREEN;
    }

    private void allKickersDown() {
        servos.kicker1GoDown();
        servos.kicker2GoDown();
        servos.kicker3GoDown();
    }
}
