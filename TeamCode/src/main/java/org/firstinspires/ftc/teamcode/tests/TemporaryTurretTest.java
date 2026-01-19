package org.firstinspires.ftc.teamcode.tests;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.layeredFinal.logical.TurretTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@TeleOp(name = "Turret Calibration 90deg")
public class TemporaryTurretTest extends OpMode {

    private TurretTracker tracker;
    private boolean lastA = false;
    private int maxTicksRecorded = 0;
    private int minTicksRecorded = 0;

    @Override
    public void init() {
        Follower follower = Constants.createFollower(hardwareMap);
        tracker = new TurretTracker(hardwareMap, follower);
        telemetry.addData("Status", "Ready - 90° turret range");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Reset encoder with A button
        if (gamepad1.a && !lastA) {
            tracker.resetEncoder();
            maxTicksRecorded = 0;
            minTicksRecorded = 0;
            telemetry.addData("Action", "ENCODER RESET!");
        }
        lastA = gamepad1.a;

        int currentTicks = tracker.encoder.getCurrentPosition();

        // Track max/min
        if (currentTicks > maxTicksRecorded) maxTicksRecorded = currentTicks;
        if (currentTicks < minTicksRecorded) minTicksRecorded = currentTicks;

        // Manual control with D-pad (SLOW for precision)
        if (gamepad1.dpad_right) {
            tracker.encoder.setPower(0.25);  // Slow near limits
        } else if (gamepad1.dpad_left) {
            tracker.encoder.setPower(-0.25);
        } else {
            tracker.encoder.setPower(0);
        }

        // Display data
        telemetry.addData("Raw Ticks", currentTicks);
        telemetry.addLine();
        telemetry.addData("Max Ticks Seen", maxTicksRecorded);
        telemetry.addData("Min Ticks Seen", minTicksRecorded);
        telemetry.addData("Total Range", maxTicksRecorded - minTicksRecorded);
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("A Button", "Reset Encoder at 0°");
        telemetry.addData("D-Pad Right", "→ Rotate Right");
        telemetry.addData("D-Pad Left", "← Rotate Left");
        telemetry.addLine();
        telemetry.addData("⚠ WARNING", "Stop before limit switches!");
        telemetry.update();
    }
}