package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import android.content.ContentValues;
import android.content.Context;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;

/**
 * A persistent storage handler that saves Limelight detections to an SQLite database.
 * This allows the team to analyze tag frequency or debugging data after the match.
 */
public class LimelightDatabase extends SQLiteOpenHelper {
    private static final String DATABASE_NAME = "limelight_data.db";
    private static final int DATABASE_VERSION = 1;

    /**
     * Initializes the database helper.
     * Note: Uses getExternalFilesDir to ensure the DB is saved on the Control Hub's
     * accessible user memory rather than internal system folders.
     */
    public LimelightDatabase(Context context) {
        // Path: /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/limelight_data.db
        super(context, context.getExternalFilesDir(null) + "/" + DATABASE_NAME, null, DATABASE_VERSION);
    }

    /**
     * Called the first time the database is created on the Control Hub.
     * Defines the table schema for storing detections.
     */
    @Override
    public void onCreate(SQLiteDatabase db) {
        // Create a table with an auto-incrementing primary key, the Tag ID,
        // and an automatic timestamp of when the detection occurred.
        db.execSQL("CREATE TABLE IF NOT EXISTS detections (" +
                "id INTEGER PRIMARY KEY AUTOINCREMENT," +
                "tag_id INTEGER," +
                "timestamp DATETIME DEFAULT CURRENT_TIMESTAMP)");
    }

    /**
     * Handles database schema changes.
     * Currently wipes the data and restarts on a version bump.
     */
    @Override
    public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {
        db.execSQL("DROP TABLE IF EXISTS detections");
        onCreate(db);
    }

    /**
     * Inserts a new AprilTag detection record into the 'detections' table.
     * @param tagId The integer ID of the AprilTag detected by the Limelight.
     */
    public void logID(int tagId) {
        // Safety check: -1 usually indicates no target found in the Limelight API
        if (tagId == -1) return;

        // Open connection for writing
        SQLiteDatabase db = this.getWritableDatabase();

        // Prepare data for insertion (Key = Column Name, Value = Data)
        ContentValues values = new ContentValues();
        values.put("tag_id", tagId);

        // Execute the insert command
        db.insert("detections", null, values);

        // Close the database to prevent memory leaks and locks
        db.close();
    }
}
