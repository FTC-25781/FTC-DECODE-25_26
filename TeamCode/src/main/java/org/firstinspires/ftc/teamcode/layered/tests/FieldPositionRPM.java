package org.firstinspires.ftc.teamcode.layered.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp(name = "Field Position RPM", group = "tests")
public class FieldPositionRPM extends LinearOpMode {

    private DcMotorEx shooter;

    // Define your setpoints: {x, y, targetRPM}
    private static final double[][] SETPOINTS = {
            {0, 60, 3000},      // Position 1: x=0", y=60", rpm=3000
            {24, 60, 3200},     // Position 2: x=24", y=60", rpm=3200
            {48, 60, 3500},     // Position 3: x=48", y=60", rpm=3500
            {-24, 48, 2900},    // Position 4: x=-24", y=48", rpm=2900
            {24, 72, 3300}      // Position 5: x=24", y=72", rpm=3300
    };

    private int currentSetpoint = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        waitForStart();

        while (opModeIsActive()) {
            // Cycle through setpoints with D-pad
            if (gamepad1.dpad_up) {
                currentSetpoint = (currentSetpoint + 1) % SETPOINTS.length;
                sleep(200); // Simple debounce
            }
            if (gamepad1.dpad_down) {
                currentSetpoint = (currentSetpoint - 1 + SETPOINTS.length) % SETPOINTS.length;
                sleep(200);
            }

            // Get current setpoint data
            double x = SETPOINTS[currentSetpoint][0];
            double y = SETPOINTS[currentSetpoint][1];
            double rpm = SETPOINTS[currentSetpoint][2];

            // Set shooter to target RPM
            double velocity = rpmToTicksPerSecond(rpm);
            shooter.setVelocity(velocity);

            // Display info
            telemetry.addData("Setpoint", "%d of %d", currentSetpoint + 1, SETPOINTS.length);
            telemetry.addData("Position", "X: %.0f\", Y: %.0f\"", x, y);
            telemetry.addData("Target RPM", "%.0f", rpm);
            telemetry.addData("Current RPM", "%.0f", getShooterRPM());
            telemetry.update();
        }

        shooter.setPower(0);
    }

    // Convert RPM to ticks per second (adjust TICKS_PER_REV for your motor)
    private double rpmToTicksPerSecond(double rpm) {
        final double TICKS_PER_REV = 28.0; // GoBILDA 5202: 28, REV HD Hex: 2240
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    // Get current shooter RPM
    private double getShooterRPM() {
        final double TICKS_PER_REV = 28.0;
        double velocity = shooter.getVelocity(); // ticks per second
        return (velocity / TICKS_PER_REV) * 60.0;
    }
}
