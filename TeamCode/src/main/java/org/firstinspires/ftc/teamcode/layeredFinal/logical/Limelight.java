package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartLimelight;

/**
 * The Limelight class serves as a logical coordinator between the physical camera
 * and the SQLite database. It provides high-level methods to capture, log, and
 * retrieve AprilTag data across different OpModes.
 */
public class Limelight {
    // Reference to the physical Limelight hardware wrapper
    private final SmartLimelight limelight;

    // Reference to the SQLite database helper for persistent storage
    private final LimelightDatabase db;

    /**
     * Constructor for the Limelight logical layer.
     * @param hardwareMap The hardwareMap provided by the OpMode to initialize the sensor
     * and retrieve the Android App Context for the database.
     */
    public Limelight(HardwareMap hardwareMap) {
        this.limelight = new SmartLimelight(hardwareMap);

        // We use the hardwareMap.appContext to give SQLite access to the Android filesystem
        this.db = new LimelightDatabase(hardwareMap.appContext);
    }

    /**
     * Captures the current AprilTag ID from the camera and automatically logs
     * valid detections to the database for later retrieval.
     * @return The detected AprilTag ID, or -1 if no target is currently visible.
     */
    public int getIDAndLog() {
        int id = limelight.getAprilTagID();

        // Only log to the database if the sensor actually sees a valid tag (-1 is 'null')
        if (id != -1) {
            db.logID(id);
        }
        return id;
    }

    /**
     * Performs a raw read of the current AprilTag ID without saving it to storage.
     * Use this for real-time tracking where persistent logging is not required.
     * @return The detected AprilTag ID.
     */
    public int getID() {
        return limelight.getAprilTagID();
    }

    /**
     * Retrieves the most recent successful AprilTag ID stored in the database.
     * This allows the robot to "remember" what it saw during Autonomous even
     * after the OpMode has been restarted for TeleOp.
     * @return The last logged ID, or -1 if no data is found.
     */
    public int getLastLoggedID() {
        return db.getLatestID();
    }

    /**
     * Properly shuts down the Limelight hardware and closes the database connection.
     * Must be called in the stop() or end of the runOpMode() to prevent database
     * memory leaks or locked file errors.
     */
    public void stop() {
        limelight.stop();
        db.close();
    }
}
