package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;

public class LimelightDatabase extends SQLiteOpenHelper {
    private static final String DATABASE_NAME = "limelight_data.db";
    private static final int DATABASE_VERSION = 1;

    public LimelightDatabase(Context context) {
        super(context, context.getExternalFilesDir(null) + "/" + DATABASE_NAME, null, DATABASE_VERSION);
    }

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

    @Override
    public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {
        db.execSQL("DROP TABLE IF EXISTS detections");
        onCreate(db);
    }

    public void logID(int tagId) {
        if (tagId == -1) return;

        SQLiteDatabase db = this.getWritableDatabase();
        ContentValues values = new ContentValues();
        values.put("tag_id", tagId);
        db.insert("detections", null, values);
        db.close();
    }

    public int getLatestID() {
        int lastId = -1;

        SQLiteDatabase db = this.getReadableDatabase();
        Cursor cursor = db.rawQuery("SELECT tag_id FROM detections ORDER BY id DESC LIMIT 1", null);

        if (cursor != null && cursor.moveToFirst()) {
            lastId = cursor.getInt(0);
            cursor.close();
        }

        db.close();
        return lastId;
    }
}
