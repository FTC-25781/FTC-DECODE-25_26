package org.firstinspires.ftc.teamcode.layered.physical1;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Test")
public class Limelight extends LinearOpMode {
    private Limelight3A limelight;

    private static final double CAMERA_HEIGHT_CM = 25.4; // 10in
    private static final double CAMERA_ANGLE_DEG = 25.0;
    private static final double CAMERA_OFFSET_CM = 0;
    private static final double APRILTAG_HEIGHT_CM = 66.04; // 26 in

    @Override
    public void runOpMode()
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double ty = result.getTy();  // Vertical offset in degrees
                double distance = calculateDistance(ty);

                telemetry.addData("APRILTAG", "DETECTED");
                telemetry.addData("Distance", "%.1f cm", distance);
                telemetry.addData("X Offset", "%.1f deg", result.getTx());
            } else {
                telemetry.addData("APRILTAG", "Not Detected");
            }

            telemetry.update();
        }
    }

    private double calculateDistance(double tyOffset) {
        // Angle to target
        double angleToTarget = CAMERA_ANGLE_DEG + tyOffset;

        // Slant distance using sin
        double distanceFromCamera = (APRILTAG_HEIGHT_CM - CAMERA_HEIGHT_CM)
                / Math.sin(Math.toRadians(angleToTarget));

        // Horizontal distance
        double horizontalDistance = Math.sqrt(
                Math.pow(distanceFromCamera, 2)
                        - Math.pow(APRILTAG_HEIGHT_CM - CAMERA_HEIGHT_CM, 2));

        // Add camera offset
        double distanceFromRobot = horizontalDistance + CAMERA_OFFSET_CM;

        return distanceFromRobot;
    }
}