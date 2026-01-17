package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class TurretTracker {

    public DcMotorEx encoder;
    public final double TICKS_PER_REV = 364;
    public Follower follower;
    public final double blueX = 12;
    public final double blueY = 132;
    public final double redX = 132;
    public final double redY = 132;
    public boolean isRed = false;
    /*
    public double tx = 0;
    public boolean targetVisible = false;
    public Limelight3A limelight;
     */


    public TurretTracker(HardwareMap hardwareMap, Follower follower){
        encoder = hardwareMap.get(DcMotorEx.class, "tmot");
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setDirection(DcMotorEx.Direction.REVERSE);
        this.follower = follower;

        /*
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
         */
    }

    public double getTurretAngle(){
        return encoder.getCurrentPosition() * (2 * Math.PI) / TICKS_PER_REV;
    }
    public void resetEncoder() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    /*
    public void limelightData() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                double rawTx = Math.toRadians(fiducials.get(0).getTargetXDegrees());
                rawTx = Math.IEEEremainder(rawTx, 2 * Math.PI);
                if (!targetVisible) {
                    tx = rawTx;
                    targetVisible = true;
                } else {
                    double delta = Math.abs(rawTx - tx);
                    double dynamicGain = (delta > Math.toRadians(0.5)) ? 0.6 : 0.15;
                    tx = (rawTx * dynamicGain) + (tx * (1 - dynamicGain));
                }
            } else {targetVisible = false;}
        } else {targetVisible = false;}
    }
     */

    public double getAngleToGoal(){
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        if(isRed){
            return Math.atan2(redY - robotY, redX - robotX);
        }
        else{
            return Math.atan2(blueY - robotY, blueX - robotX);
        }
    }
    public double calculateDesiredTurretAngle(){
        double robotHeading = follower.getPose().getHeading();
        return getAngleToGoal() - robotHeading;
    }
    public double calculateError() {
        double error = calculateDesiredTurretAngle() - getTurretAngle();
        return Math.IEEEremainder(error, 2 * Math.PI);
    }
}
