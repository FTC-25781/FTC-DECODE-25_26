package org.firstinspires.ftc.teamcode.layeredFinal.physical;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;

/**
 * Physical layer wrapper for the Limelight 3A vision sensor.
 * This class handles the initialization and data extraction for AprilTag detection.
 */
public class SmartLimelight {
    private Limelight3A limelight;

    /**
     * Initializes the Limelight hardware.
     * @param hardwareMap The hardware map from the OpMode.
     */
    public SmartLimelight(HardwareMap hardwareMap) {
        // "limelight" must match the name in your Driver Station configuration
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Set the active pipeline (0 should be your AprilTag pipeline in the LL dashboard)
        limelight.pipelineSwitch(0);

        // Starts the camera stream and processing
        limelight.start();
    }

    /**
     * Scans the current camera frame for AprilTags and retrieves the ID of the primary target.
     * @return The ID of the first detected tag, or -1 if no tags are visible.
     */
    public int getAprilTagID() {
        // Capture the most recent snapshot of data from the Limelight
        LLResult result = limelight.getLatestResult();

        // Safety check: Ensure the camera is providing data and the frame is valid
        if (result != null && result.isValid()) {
            // Get the list of all "Fiducial" (AprilTag) results in the current frame
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            // Check if the camera has successfully identified at least one tag
            if (!fiducialResults.isEmpty()) {
                // Returns the ID of the first tag in the list (usually the most prominent).
                // We cast to (int) because ID values are typically stored as doubles/longs in LL.
                return (int) fiducialResults.get(0).getFiducialId();
            }
        }

        // Return -1 to signify "No Target Found"
        return -1;
    }

    /**
     * Stops the Limelight to save power/processing when vision is no longer needed.
     */
    public void stop() {
        limelight.stop();
    }
}
