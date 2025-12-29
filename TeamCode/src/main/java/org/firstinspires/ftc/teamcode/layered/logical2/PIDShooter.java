package org.firstinspires.ftc.teamcode.layered.logical2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.layered.physical1.FlywheelMotor;

@Disabled
@TeleOp(name="PID Shooter")
public class PIDShooter extends LinearOpMode {
    public FlywheelMotor flywheel;
    public GetDistanceToGoal distanceCalculation;
    public Follower follower;

    public Pose startingPose = new Pose(72, 72, Math.toRadians(135));

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            double distance = distanceCalculation.getDistanceToGoal();

            // --- LIVE TUNING LOGIC ---
            // Using a small sleep or specific logic to prevent values from "runaway"
            if (gamepad1.dpad_up) {
                FlywheelMotor.NEAR_EFFICIENCY += 0.005;
                sleep(50); // Small delay so one tap = one increment
            } else if (gamepad1.dpad_down) {
                FlywheelMotor.NEAR_EFFICIENCY -= 0.005;
                sleep(50);
            }

            if (gamepad1.y) { // Using Y/A for Far Efficiency to keep D-pad free
                FlywheelMotor.FAR_EFFICIENCY += 0.005;
                sleep(50);
            } else if (gamepad1.a) {
                FlywheelMotor.FAR_EFFICIENCY -= 0.005;
                sleep(50);
            }


            // --- MOTOR COMMAND ---
            double targetRPM = flywheel.targetRPM(distance);

            // EMERGENCY STOP: Hold B to kill the motor
            if (gamepad1.b) {
                flywheel.setRPM(0);
            } else {
                flywheel.setRPM(targetRPM);
            }

            // --- TELEMETRY ---
            telemetry.addLine("== EFFICIENCY TUNER ==");
            telemetry.addData("NEAR (2m)", "%.4f", FlywheelMotor.NEAR_EFFICIENCY);
            telemetry.addData("FAR (4.5m)", "%.4f", FlywheelMotor.FAR_EFFICIENCY);
            telemetry.addLine("--------------------");
            telemetry.addData("Distance (in)", "%.2f", distanceCalculation.getDistanceInInches());
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Actual RPM", "%.0f", flywheel.getCurrentRPM());
            telemetry.addData("Ready", flywheel.isShooterReady() ? "!!! READY !!!" : "Spinning up...");
            telemetry.update();
        }
        flywheel.stop();
    }

    public void initHardware() {
        flywheel = new FlywheelMotor(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap); // Use standard init if Constants gives trouble
        follower.setStartingPose(startingPose);
        distanceCalculation = new GetDistanceToGoal(follower, true, telemetry);
    }
}