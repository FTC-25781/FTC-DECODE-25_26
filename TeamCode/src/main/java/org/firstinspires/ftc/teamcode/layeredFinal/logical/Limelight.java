package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartLimelight;

/**
 * High-level logical wrapper for Limelight operations.
 * This class abstracts the physical implementation (SmartLimelight)
 * to provide a simpler interface for the robot's main logic.
 */
public class Limelight {
    // Physical hardware interface for the Limelight sensor
    private final SmartLimelight limelight;

    // SQLite database handler for persisting Limelight data
    private final LimelightDatabase db;

    /**
     * Initializes the Limelight hardware and the local database.
     * @param hardwareMap The hardware map from the OpMode to locate the sensor and app context.
     */
    public Limelight(HardwareMap hardwareMap) {
        // Initialize the physical sensor wrapper
        limelight = new SmartLimelight(hardwareMap);

        // Initialize database using the Android App Context for file storage access
        db = new LimelightDatabase(hardwareMap.appContext);
    }

    /**
     * Retrieves the current AprilTag ID from the sensor and saves it to the database.
     * Only logs to the database if a valid ID (not -1) is detected.
     * @return The detected AprilTag ID, or -1 if no tag is in view.
     */
    public int getIDAndLog() {
        int id = limelight.getAprilTagID();

        // If a valid tag is detected, record it in the local database
        if (id != -1) {
            db.logID(id);
        }
        return id;
    }

    /**
     * Retrieves the current AprilTag ID without saving to the database.
     * Use this for high-frequency loops where logging every cycle isn't necessary.
     * @return The detected AprilTag ID.
     */
    public int getID() {
        return limelight.getAprilTagID();
    }

    /**
     * Safely shuts down the Limelight hardware and closes the database connection.
     * Should be called in the 'stop' phase of the OpMode to prevent memory leaks or DB corruption.
     */
    public void stop() {
        limelight.stop();
        db.close(); // Ensures the SQLite connection is properly terminated
    }
}
