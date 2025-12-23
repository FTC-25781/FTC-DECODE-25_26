package org.firstinspires.ftc.teamcode.layered.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.layered.physical1.FlywheelMotor;

@TeleOp(name="Flywheel PIDF Quick Tuner", group="test")
public class FlywheelPIDFQuickTuner extends LinearOpMode {

    FlywheelMotor flywheel;
    VoltageSensor battery;
    double targetRPM = 3400; // starting value

    @Override
    public void runOpMode() throws InterruptedException {

        flywheel = new FlywheelMotor(hardwareMap, telemetry);
        battery = hardwareMap.voltageSensor.iterator().next();

        telemetry.addLine("A/B to increase/decrease RPM");
        telemetry.addLine("D-pad Up/Down = kP, Right/Left = kD");
        telemetry.addLine("Left/Right Bumper = kI, Y/X = kF");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Adjust target RPM
            if (gamepad1.a) targetRPM += 10;
            if (gamepad1.b) targetRPM -= 10;

            // Live PIDF adjustments
            if (gamepad1.dpad_up) FlywheelMotor.kP += 0.1;
            if (gamepad1.dpad_down) FlywheelMotor.kP -= 0.1;
            if (gamepad1.dpad_right) FlywheelMotor.kD += 0.1;
            if (gamepad1.dpad_left) FlywheelMotor.kD -= 0.1;
            if (gamepad1.right_bumper) FlywheelMotor.kI += 0.001;
            if (gamepad1.left_bumper) FlywheelMotor.kI -= 0.001;
            if (gamepad1.y) FlywheelMotor.kF += 0.1;
            if (gamepad1.x) FlywheelMotor.kF -= 0.1;

            // Update motor PIDF
            flywheel.updatePIDFCoefficients();
            flywheel.setRPM(targetRPM);

            // Telemetry for tuning
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Current RPM", "%.0f", flywheel.getCurrentRPM());
            telemetry.addData("Error", "%.0f", targetRPM - flywheel.getCurrentRPM());
            telemetry.addData("Shooter Ready", flywheel.isShooterReady());
            telemetry.addData("Battery Voltage", "%.2fV", battery.getVoltage());
            telemetry.addData("kP/I/D/F", "%.2f / %.4f / %.2f / %.2f",
                    FlywheelMotor.kP, FlywheelMotor.kI, FlywheelMotor.kD, FlywheelMotor.kF);
            telemetry.update();

            sleep(50);
        }

        flywheel.stop();
    }
}
