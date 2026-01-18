package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Turret {

    public TurretTracker turretOrientation;
    public PIDFController turretPID;
    public boolean autoAlign = false;
    public boolean redAlliance = true;

    public double kP = 0.012;
    public double kI = 0.0;
    public double kD = 0.003;
    public double kF = 0.0008;

    public double angleTolerance = 2.0;
    public double settleZone = 1.0;

    static final double TICKS_PER_180_DEG = 171;
    static final double DEGREES_PER_180_TICKS = 180.0;
    static final double TICKS_PER_DEGREE = TICKS_PER_180_DEG / DEGREES_PER_180_TICKS;
    public static final int MAX_TICKS = 85;
    public static final int MIN_TICKS = -88;

    private double minPower = 0.1;
    private double maxPower = 0.6;

    private long lastUpdateTime = 0;
    private final long UPDATE_INTERVAL_MS = 30;

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

    public void stopAutoAlign() {
        autoAlign = false;
        turretOrientation.encoder.setPower(0);
    }

    private int degreesToTicks(double degrees) {
        return (int) Math.round(degrees * TICKS_PER_DEGREE);
    }

    private double normalizeAngle(double angle) {
        return Math.IEEEremainder(angle, 360.0);
    }

    public void update() {
        if (!autoAlign) return;


        long currentTime = System.currentTimeMillis();
        if (currentTime - lastUpdateTime < UPDATE_INTERVAL_MS) {
            return;
        }
        lastUpdateTime = currentTime;

        int currentTicks = turretOrientation.encoder.getCurrentPosition();
        if(currentTicks <= MIN_TICKS || currentTicks >= MAX_TICKS){
            turretOrientation.encoder.setPower(0);
            return;
        }


        double desiredTurretAngleDeg = turretOrientation.calculateDesiredTurretAngle();
        double currentTurretDeg = turretOrientation.getTurretAngle();
        double errorDeg = normalizeAngle(desiredTurretAngleDeg - currentTurretDeg);

        if (Math.abs(errorDeg) <= settleZone) {
            turretOrientation.encoder.setPower(0);
            return;
        }

        int targetTicks;
        targetTicks = currentTicks + degreesToTicks(errorDeg);

        turretPID.setTargetPosition(targetTicks);
        turretPID.updatePosition(currentTicks);
        double pidOutput = turretPID.run();

        double power = Range.clip(pidOutput, -maxPower, maxPower);

        if (Math.abs(power) > 0 && Math.abs(power) < minPower) {
            power = Math.signum(power) * minPower;
        }

        turretOrientation.encoder.setPower(power);
    }

    public boolean isOnTarget() {
        double desiredTurretAngleDeg = turretOrientation.calculateDesiredTurretAngle();
        double currentTurretDeg = turretOrientation.getTurretAngle();
        double error = normalizeAngle(desiredTurretAngleDeg - currentTurretDeg);
        return Math.abs(error) <= angleTolerance;
    }
}