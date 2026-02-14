package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartLimelight;

public class Limelight {
    private final SmartLimelight limelight;
    private final LimelightDatabase db;
    boolean isRed = true;

    public Limelight(HardwareMap hardwareMap) {
        this.limelight = new SmartLimelight(hardwareMap);
        this.db = new LimelightDatabase(hardwareMap.appContext);
    }

    public int getIDAndLog(int id) {
        if (id != -1) {
            if (id == 21 || id == 22 || id == 23) {
                db.logID(id);
            }
        }
        return id;
    }
    public double getAprilTagTargetX(){
        if(isRed){
            return limelight.getTargetX(22); //TODO: Change target ID values based on the real ones
        } else {
            return limelight.getTargetX(21);
        }
    }

    public int getID() {
        return limelight.getAprilTagID();
    }

    public int getLastLoggedID() {
        return db.getLatestID();
    }

    public void stop() {
        limelight.stop();
        db.close();
    }
}
