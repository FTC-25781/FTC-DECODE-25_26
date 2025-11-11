package org.firstinspires.ftc.teamcode.layered.physical1;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Test", group = "Test")
public class Limelight extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Get the Limelight camera from the configuration
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            // Request the latest result
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double ty = result.getTy(); // Vertical offset angle
                double tx = result.getTx(); // Horizontal offset angle
                double ta = result.getTa(); // Target area (optional)

                // Limelight and target parameters
                double limelightMountAngleDegrees = 25.0;
                double limelightLensHeightInches = 11.5;
                double goalHeightInches = 26.0;

                // Calculate distance using vertical offset
                double angleToGoalDegrees = limelightMountAngleDegrees + ty;
                double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

                double distanceFromLimelightToGoalInches =
                        (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("ta", ta);
                telemetry.addData("Distance (in)", distanceFromLimelightToGoalInches);
            } else {
                telemetry.addData("Status", "No valid target");
            }

            telemetry.update();
        }
    }
}
