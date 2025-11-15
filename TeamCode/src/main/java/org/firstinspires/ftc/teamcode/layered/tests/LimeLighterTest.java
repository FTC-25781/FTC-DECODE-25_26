package org.firstinspires.ftc.teamcode.layered.tests;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

@Configurable
@TeleOp(name = "LimeLighterTest", group = "TeleOp")
public class LimeLighterTest extends LinearOpMode {

    private Limelight3A limelight;

    ArrayList<Integer> tags = new ArrayList<Integer>();

    static int obeliskValue = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        limelight.start();

        tags.add(21);
        tags.add(22);
        tags.add(23);

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    /*
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("ta --> Target Area", result.getTa());
                    telemetry.addData("Botpose", botpose.toString());
                     */
                }
            }

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId(); // The ID number of the fiducial
                double StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getPosition().y;
                telemetry.addData("Fiducial " + id, "is " + StrafeDistance_3D + " meters away");
                if(!tags.contains(obeliskValue)){
                    if(tags.contains(id)){
                        obeliskValue = id;
                    }
                }
            }

            telemetry.addData("Obelisk Value", obeliskValue);
            telemetry.update();
        }
    }
}
