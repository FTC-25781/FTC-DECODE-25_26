package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartLimelight;

import java.util.List;

public class TurretTracker {

    public DcMotorEx encoder;
    public SmartLimelight limelight;
    public static final double TICKS_PER_180_DEG = 171;
    public static final double DEGREES_PER_180_TICKS = 180.0;
    public static final double DEGREES_PER_TICK = DEGREES_PER_180_TICKS / TICKS_PER_180_DEG;

    //public static final double OFFSET_DEG = 12.48;
    public double tx = 0;
    public boolean targetVisible = false;

    public Follower follower;

    public final double blueX = 0;
    public final double blueY = 144;

    public final double redX = 144;
    public final double redY = 144;

    public boolean isRed = true;

    public TurretTracker(HardwareMap hardwareMap, Follower follower){
        encoder = hardwareMap.get(DcMotorEx.class, "tmot");

        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        encoder.setDirection(DcMotorEx.Direction.REVERSE);

        limelight = new SmartLimelight(hardwareMap);


        this.follower = follower;
    }

    public double getTurretAngle(){
        return encoder.getCurrentPosition() * DEGREES_PER_TICK;
    }

    public void resetEncoder() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getAngleToGoal(){
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        double targetX = isRed ? redX : blueX;
        double targetY = isRed ? redY : blueY;

        double angleRad = Math.atan2(targetY - robotY, targetX - robotX);
        return Math.toDegrees(angleRad);
    }

    public double calculateDesiredTurretAngle(){
        double robotHeadingRad = follower.getPose().getHeading();
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        return (getAngleToGoal() - robotHeadingDeg);
    }
    public double calculateError(){
        return calculateDesiredTurretAngle() - getTurretAngle();
    }
    public void limelightData(LLResult result) {
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                double rawTx = fiducials.get(0).getTargetXDegrees();
                if (!targetVisible) {
                    tx = rawTx;
                    targetVisible = true;
                } else {
                    double delta = Math.abs(rawTx - tx);
                    double dynamicGain = (delta > 0.5) ? 0.6 : 0.15;
                    tx = (rawTx * dynamicGain) + (tx * (1 - dynamicGain));
                }
            } else { targetVisible = false; }
        } else { targetVisible = false; }
    }
}