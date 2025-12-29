package org.firstinspires.ftc.teamcode.layered.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.layered.physical1.FlywheelMotor;

@Disabled
@TeleOp(name="Flywheel Efficiency Tuner", group="Tuning")
public class FlywheelTuningTeleOp extends LinearOpMode {
    private FlywheelMotor flywheel;
    private Follower follower;

    // Field coordinates for your goal
    private double goalX = 12.0;
    private double goalY = 132.0;

    private double manualRPM = 3000;

    @Override
    public void runOpMode() {
        flywheel = new FlywheelMotor(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(72, 72, Math.toRadians(135)));

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            // 1. Controls
            if (gamepad1.dpad_up) manualRPM += 5;
            if (gamepad1.dpad_down) manualRPM -= 5;
            if (gamepad1.b) manualRPM = 0;

            flywheel.setRPM(manualRPM);

            // 2. Distance Calculation
            Pose currentPose = follower.getPose();
            double dx = goalX - currentPose.getX();
            double dy = goalY - currentPose.getY();
            double distanceInches = Math.sqrt(dx*dx + dy*dy);
            double distanceMM = distanceInches * 25.4;

            // 3. Efficiency Calculation
            // What the physics engine says the ball needs (velocity in mm/s)
            double idealBallVel = flywheel.calculateRequiredRPM(distanceMM);

            // What your wheel is actually doing (tangential velocity in mm/s)
            double actualRPM = Math.abs(flywheel.getCurrentRPM());
            double wheelCircumference = Math.PI * FlywheelMotor.WHEEL_DIAMETER_MM;
            double actualWheelVel = (actualRPM * wheelCircumference) / 60.0;

            // Efficiency = (Ball Velocity) / (Wheel Velocity)
            double liveEfficiency = (actualWheelVel == 0) ? 0 : idealBallVel / actualWheelVel;

            // 4. Telemetry
            telemetry.addLine("--- DRIVE & SCORE TO TUNE ---");
            telemetry.addData("Dist to Goal (mm)", "%.1f", distanceMM);
            telemetry.addData("Manual RPM", "%.0f", manualRPM);
            telemetry.addData("Actual RPM", "%.0f", actualRPM);
            telemetry.addLine("--- RESULTS ---");
            telemetry.addData("REQUIRED EFFICIENCY", "%.4f", liveEfficiency);
            telemetry.addLine("\nCopy this value for this distance into FlywheelMotor.java");
            telemetry.update();
        }
    }
}
