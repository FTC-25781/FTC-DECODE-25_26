package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

public class Turret {

    public TurretTracker turretOrientation;
    public PIDFController turretPID;
    public boolean autoAlign = false;
    public boolean redAlliance = true;
    public static double RED_OFFSET = 15.4948;
    public static double BLUE_OFFSET = -14.5106;

    public static final int MAX_TICKS = 182;
    public static final int MIN_TICKS = -182;

    public double kP = 0.04;
    public double kI = 0.0;
    public double kD = 0.0009;
    public double kF = 0.000;

    public double angleTolerance = 0.7;
    public double direction = 1;

    /*
    static final double TICKS_PER_180_DEG = 182;
    static final double DEGREES_PER_180_TICKS = 180.0;
     */
    static final double TICKS_PER_DEGREE = 182 / 180.0;

    private double minPower = 0.1;
    private double maxPower = 0.6;

    public Turret(Follower follower, HardwareMap hardwareMap) {
        this.turretOrientation = new TurretTracker(hardwareMap, follower);
        turretPID = new PIDFController(new PIDFCoefficients(kP, kI, kD, kF));
    }

    public void setAlliance(boolean red) {
        this.redAlliance = red;
        this.turretOrientation.isRed = red;
    }
    public void startAutoAlign() {
        autoAlign = true;
    }
    private double normalizeAngle(double angle) {
        return Math.IEEEremainder(angle, 360.0);
    }

    public void update() {
        if (!autoAlign) return;

        double worldTarget = turretOrientation.getAngleToGoal();
        double robotHeading = Math.toDegrees(turretOrientation.follower.getPose().getHeading());



        double relativeTarget = worldTarget - robotHeading;

        relativeTarget += (redAlliance ? RED_OFFSET : BLUE_OFFSET);

        while (relativeTarget > 180) relativeTarget -= 360;
        while (relativeTarget <= -180) relativeTarget += 360;

        double clampedTargetDeg = Range.clip(relativeTarget,
                MIN_TICKS * TurretTracker.DEGREES_PER_TICK,
                MAX_TICKS * TurretTracker.DEGREES_PER_TICK);

        int targetTicks = (int) Math.round(clampedTargetDeg / TurretTracker.DEGREES_PER_TICK);

        int currentTicks = turretOrientation.encoder.getCurrentPosition();
        turretPID.setTargetPosition(targetTicks);
        turretPID.updatePosition(currentTicks);

        double power = turretPID.run();

        if ((currentTicks >= MAX_TICKS && power > 0) || (currentTicks <= MIN_TICKS && power < 0)) {
            power = 0;
        }

        if (Math.abs(power) > 0 && Math.abs(power) < minPower) {
            power = Math.signum(power) * minPower;
        }

        turretOrientation.encoder.setPower(Range.clip(power, -maxPower, maxPower));
    }
    public boolean isOnTarget() {
        double desiredTurretAngleDeg = turretOrientation.calculateDesiredTurretAngle();
        double currentTurretDeg = turretOrientation.turretLocalAngle();
        double error = normalizeAngle(desiredTurretAngleDeg - currentTurretDeg);
        return Math.abs(error) <= angleTolerance;
    }

}