package org.firstinspires.ftc.teamcode.layered.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.layered.physical1.Motor;
import org.firstinspires.ftc.teamcode.layered.logical2.PIDShooter;
import Layered.PhysicalLayer.SmartServo;

@TeleOp(name = "Shooter RPM + Angler Test", group = "Test")
public class PIDShooterTest extends LinearOpMode {

    // Shooter
    private Motor shooterMotor;
    private SmartServo angleServo;
    private PIDShooter shooter;
    private IMU imu;

    // State
    private boolean isRedAlliance = false; // false = BLUE, true = RED
    private long lastButtonPress = 0;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        try {
            // Shooter motor
            VoltageSensor voltage = hardwareMap.voltageSensor.iterator().next();
            DcMotorEx rawMotor = hardwareMap.get(DcMotorEx.class, "dmot");
            Servo rawServo = hardwareMap.get(Servo.class, "transfer2");

            shooterMotor = new Motor(rawMotor, voltage);
            shooterMotor.setDirection(DcMotor.Direction.FORWARD);
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterMotor.setVoltageCap(12);
            telemetry.addData("✓ Shooter Motor", "dmot");

            angleServo = new SmartServo(rawServo, SmartServo.ServoType.STANDARD_180);
            telemetry.addData("✓ Angle Servo", "transfer2");

            // Optional IMU
            try {
                imu = hardwareMap.get(IMU.class, "imu");
                telemetry.addData("✓ IMU", "Found");
            } catch (Exception e) {
                imu = null;
                telemetry.addData("⚠ IMU", "Not found (pitch compensation disabled)");
            }

            // Initialize PIDShooter
            shooter = new PIDShooter(shooterMotor, angleServo, imu);
            telemetry.addData("✓ PIDShooter", "Ready");

            telemetry.addLine();
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addLine();
            telemetry.addLine("CONTROLS:");
            telemetry.addLine("  B - Switch Alliance");
            telemetry.addLine("  X - Increase RPM target");
            telemetry.addLine("  A - Decrease RPM target");
            telemetry.addLine("  Y - Adjust Servo Angle");
            telemetry.addLine();
            telemetry.addLine("Ready! Press PLAY");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("INIT FAILED", e.getMessage());
            telemetry.addData("Stack", e.getStackTrace()[0].toString());
            telemetry.update();
            while (!isStopRequested()) sleep(100);
            return;
        }

        waitForStart();

        // Shooter tuning variables
        double targetRPM = 3000;
        double servoPos = 0.5;

        while (opModeIsActive()) {
            try {
                handleButtons();


                if (gamepad1.x) targetRPM += 50;
                if (gamepad1.a) targetRPM -= 50;
                targetRPM = Math.max(0, Math.min(targetRPM, 6000));


                if (gamepad1.y) servoPos += 0.01;
                if (gamepad1.dpad_down) servoPos -= 0.01;
                servoPos = Math.max(0, Math.min(1, servoPos));
                angleServo.setPosition(servoPos);


                shooter.setTargetRPM(targetRPM);
                shooter.update(telemetry, 0, 0, 0, isRedAlliance);


                telemetry.addLine("═══ SHOOTER TEST ═══");
                telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
                telemetry.addData("Target RPM", "%.0f", targetRPM);
                telemetry.addData("Current RPM", "%.0f", shooter.getCurrentRPM());
                telemetry.addData("At Target?", shooter.isAtTargetSpeed(50) ? "✓ YES" : "NO");
                telemetry.addData("Servo Pos", "%.2f", servoPos);
                telemetry.update();

            } catch (Exception e) {
                telemetry.addData("ERROR", e.getMessage());
                telemetry.update();
            }

            sleep(50);
        }

        shooter.stop();
    }

    private void handleButtons() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastButtonPress < 300) return;

        if (gamepad1.b) {
            isRedAlliance = !isRedAlliance;
            lastButtonPress = currentTime;
        }
    }
}
