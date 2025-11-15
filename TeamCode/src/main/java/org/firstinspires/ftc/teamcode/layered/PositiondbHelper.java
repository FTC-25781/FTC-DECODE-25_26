package org.firstinspires.ftc.teamcode.layered;

import android.database.sqlite.SQLiteOpenHelper;
import android.content.Context;
import android.database.sqlite.SQLiteDatabase;

public class PositiondbHelper extends SQLiteOpenHelper {
    public static final int DATABASE_VERSION = 1;
    public static final String DATABASE_NAME = "apriltags.db";

    private static final String SQL_CREATE_ENTRIES =
            "CREATE TABLE " + PositionContract.PositionEntry.TABLE_NAME + " (" +
                    PositionContract.PositionEntry.COLUMN_NAME_X + " REAL," +
                    PositionContract.PositionEntry.COLUMN_NAME_Y + " REAL," +
                    PositionContract.PositionEntry.COLUMN_NAME_HEADING + " REAL)";
    private static final String SQL_DELETE_ENTRIES =
            "DROP TABLE IF EXISTS " + PositionContract.PositionEntry.TABLE_NAME;
    public PositiondbHelper(Context context){
        super(context, DATABASE_NAME, null, DATABASE_VERSION);
    }
    @Override
    public void onCreate(SQLiteDatabase db){
        db.execSQL(SQL_CREATE_ENTRIES);
    }
    @Override
    public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion){
        db.execSQL(SQL_DELETE_ENTRIES);
        onCreate(db);
    }
}
