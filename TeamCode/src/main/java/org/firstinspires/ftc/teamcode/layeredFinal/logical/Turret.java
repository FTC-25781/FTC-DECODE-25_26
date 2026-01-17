package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Turret {
    public TurretTracker turretOrientation;
    public double angleTolerance = Math.toRadians(2);
    public PIDFController turretPID;
    public Turret(HardwareMap hardwareMap, Follower follower) {
        this.turretOrientation = new TurretTracker(hardwareMap, follower);
        turretPID = new PIDFController(new PIDFCoefficients(0.025, 0,0, 0));
        turretOrientation.encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretOrientation.encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretOrientation.encoder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }
    public void trackGoal() {
        double currentTicks = turretOrientation.encoder.getCurrentPosition();
        turretPID.updatePosition(currentTicks);
        double targetRadians = turretOrientation.calculateDesiredTurretAngle();
        double targetTicks = targetRadians * turretOrientation.TICKS_PER_REV / (2 * Math.PI);
        turretPID.setTargetPosition(targetTicks);
        double output = turretPID.run() / 2.0;
        double clippedOutput = Range.clip(output, -1.0, 1.0);
        double error = turretOrientation.calculateError();
        if (Math.abs(error) <= angleTolerance) {
            turretOrientation.encoder.setPower(0);
        } else {
            turretOrientation.encoder.setPower(clippedOutput);
        }
    }

    public boolean isOnTarget() {
        double currentError = turretOrientation.calculateError();
        return Math.abs(currentError) <= angleTolerance;
    }
}
