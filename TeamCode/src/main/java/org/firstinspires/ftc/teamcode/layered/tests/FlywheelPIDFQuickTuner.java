package org.firstinspires.ftc.teamcode.layered.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.layered.physical1.FlywheelMotor;

/**
 * This OpMode allows you to tune your Flywheel PIDF coefficients live using a gamepad.
 * Since constants are static, changes made here will persist in the code until the OpMode is restarted.
 */
@TeleOp(name="Flywheel PIDF Quick Tuner", group="test")
public class FlywheelPIDFQuickTuner extends LinearOpMode {

    FlywheelMotor flywheel;
    VoltageSensor battery;
    double targetRPM = 3400; // Default starting speed for testing

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the flywheel and battery sensor
        flywheel = new FlywheelMotor(hardwareMap, telemetry);
        battery = hardwareMap.voltageSensor.iterator().next();

        // Instructions for the driver displayed on the driver station
        telemetry.addLine("A/B to increase/decrease RPM");
        telemetry.addLine("D-pad Up/Down = kP, Right/Left = kD");
        telemetry.addLine("Left/Right Bumper = kI, Y/X = kF");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- RPM Adjustment ---
            // Use A and B to change the target speed to see how the motor reacts at different velocities
            if (gamepad1.a) targetRPM += 10;
            if (gamepad1.b) targetRPM -= 10;

            // --- PID Tuning Controls ---
            // P (Proportional): Increases "snapiness" and power as error increases
            if (gamepad1.dpad_up) FlywheelMotor.kP += 0.1;
            if (gamepad1.dpad_down) FlywheelMotor.kP -= 0.1;

            // D (Derivative): Dampens the movement to prevent the speed from overshooting/oscillating
            if (gamepad1.dpad_right) FlywheelMotor.kD += 0.1;
            if (gamepad1.dpad_left) FlywheelMotor.kD -= 0.1;

            // I (Integral): Fixes steady-state error (if the motor is always 50 RPM slow)
            if (gamepad1.right_bumper) FlywheelMotor.kI += 0.001;
            if (gamepad1.left_bumper) FlywheelMotor.kI -= 0.001;

            // F (Feedforward): The "base" power needed to reach the target RPM.
            // For flywheels, kF is the most important value.
            if (gamepad1.y) FlywheelMotor.kF += 0.1;
            if (gamepad1.x) FlywheelMotor.kF -= 0.1;

            // Apply the new PIDF values to the motor controller hardware
            flywheel.updatePIDFCoefficients();

            // Command the motor to the (potentially updated) target RPM
            flywheel.setRPM(targetRPM);

            // --- Tuning Feedback ---
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Current RPM", "%.0f", flywheel.getCurrentRPM());

            // Error: How far away we are from the goal. This should ideally be close to 0.
            telemetry.addData("Error", "%.0f", targetRPM - flywheel.getCurrentRPM());

            // Ready: Indicates if the motor is within the RPM_TOLERANCE defined in FlywheelMotor
            telemetry.addData("Shooter Ready", flywheel.isShooterReady());

            // Voltage is important: If battery is low, kF might need to be higher
            telemetry.addData("Battery Voltage", "%.2fV", battery.getVoltage());

            // Display the current variables so you can write them down once the shooter is stable
            telemetry.addData("kP/I/D/F", "%.2f / %.4f / %.2f / %.2f",
                    FlywheelMotor.kP, FlywheelMotor.kI, FlywheelMotor.kD, FlywheelMotor.kF);
            telemetry.update();

            // Small delay to prevent the variables from changing too fast when a button is pressed
            sleep(50);
        }

        // Safety: Stop the motor when the OpMode ends
        flywheel.stop();
    }
}