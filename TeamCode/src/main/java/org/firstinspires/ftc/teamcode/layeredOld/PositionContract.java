package org.firstinspires.ftc.teamcode.layeredOld;

public class PositionContract {
    private PositionContract(){}

    public static class PositionEntry{ // private constructor so positionentry object isn't created
        public static final String TABLE_NAME = "robot_pose";
        public static final String COLUMN_NAME_X = "x_pos";
        public static final String COLUMN_NAME_Y = "y_pos";
        public static final String COLUMN_NAME_HEADING = "heading";
    }
}
