package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;

/**
 * Handles persistent storage of AprilTag detections using an SQLite database.
 * This class ensures that data survives between OpMode restarts (e.g., from Auto to TeleOp).
 */
public class LimelightDatabase extends SQLiteOpenHelper {
    // The filename of the database stored on the Control Hub
    private static final String DATABASE_NAME = "limelight_data.db";

    // Increment this version number if you change the table schema (columns/types)
    private static final int DATABASE_VERSION = 1;

    /**
     * Initializes the database helper.
     * Uses getExternalFilesDir to ensure the database is stored in the robot's
     * accessible internal storage (/sdcard/Android/data/...), making it easier to extract.
     */
    public LimelightDatabase(Context context) {
        super(context, context.getExternalFilesDir(null) + "/" + DATABASE_NAME, null, DATABASE_VERSION);
    }

    /**
     * Called when the database is created for the first time.
     * Sets up the 'detections' table with an auto-incrementing ID and an automatic timestamp.
     */
    @Override
    public void onCreate(SQLiteDatabase db) {
        // SQL Table Schema:
        // id: Unique primary key for every row
        // tag_id: The integer ID detected by the Limelight
        // timestamp: Automatically filled with the current date/time when the row is added
        db.execSQL("CREATE TABLE IF NOT EXISTS detections (" +
                "id INTEGER PRIMARY KEY AUTOINCREMENT," +
                "tag_id INTEGER," +
                "timestamp DATETIME DEFAULT CURRENT_TIMESTAMP)");
    }

    /**
     * Handles schema updates. If the version is bumped, it wipes the old data.
     */
    @Override
    public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {
        db.execSQL("DROP TABLE IF EXISTS detections");
        onCreate(db);
    }

    /**
     * Inserts a new detection into the database.
     * @param tagId The AprilTag ID to save. If -1, the call is ignored.
     */
    public void logID(int tagId) {
        // -1 usually represents "no target found" in vision APIs
        if (tagId == -1) return;

        // Open connection for writing
        SQLiteDatabase db = this.getWritableDatabase();

        // Use ContentValues to prevent SQL injection and handle data mapping
        ContentValues values = new ContentValues();
        values.put("tag_id", tagId);

        // Execute the insert
        db.insert("detections", null, values);

        // Important: Close the connection to prevent memory leaks or file locks
        db.close();
    }

    /**
     * Retrieves the most recent Tag ID from the database using a descending sort.
     * Useful for setting the robot's state at the start of TeleOp based on Auto vision.
     * @return The last saved ID, or -1 if the table is empty.
     */
    public int getLatestID() {
        int lastId = -1;

        // Open connection for reading only
        SQLiteDatabase db = this.getReadableDatabase();

        // Query Strategy: Sort by the primary key 'id' in descending order and
        // use 'LIMIT 1' to grab only the single most recent record.
        Cursor cursor = db.rawQuery("SELECT tag_id FROM detections ORDER BY id DESC LIMIT 1", null);

        // Check if the query actually returned any rows
        if (cursor != null && cursor.moveToFirst()) {
            // Index 0 corresponds to the first column requested ('tag_id')
            lastId = cursor.getInt(0);
            cursor.close();
        }

        db.close();
        return lastId;
    }
}
