package org.firstinspires.ftc.teamcode.layeredFinal.physical;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;

public class SmartLimelight {
    public Limelight3A limelight;

    public SmartLimelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public int getAprilTagID() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            if (!fiducialResults.isEmpty()) {
                return (int) fiducialResults.get(0).getFiducialId();
            }
        }

        return -1;
    }
    public double getTargetX(int targetID) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for(LLResultTypes.FiducialResult fiducial : fiducials){
                if (fiducial.getFiducialId() == targetID) {
                    return fiducial.getTargetXDegrees();
                }
            }

        }
        return 0;
    }

    public void stop() {
        limelight.stop();
    }
}
