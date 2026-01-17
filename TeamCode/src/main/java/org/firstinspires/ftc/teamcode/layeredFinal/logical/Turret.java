package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Turret {

    public TurretTracker turretOrientation;
    public PIDFController turretPID;

    // Alliance-based auto-align
    public boolean autoAlign = false;
    public boolean redAlliance = true;

    public double kP = 0.012;
    public double kI = 0.0;
    public double kD = 0.003;
    public double kF = 0.0008;

    // Tolerances
    public double angleTolerance = 2.0; // degrees
    public double settleZone = 5.0;     // Stop moving within this range

    // Encoder conversion
    static final double TICKS_PER_180_DEG = 622;
    static final double DEGREES_PER_180_TICKS = 180.0;
    static final double TICKS_PER_DEGREE = TICKS_PER_180_DEG / DEGREES_PER_180_TICKS;

    // Power limits
    private double minPower = 0.1;   // Minimum power to overcome friction
    private double maxPower = 0.6;   // maximum speed

    // Update rate control
    private long lastUpdateTime = 0;
    private final long UPDATE_INTERVAL_MS = 30; // Update every 30ms to reduce jitter from constant pid updates

    public Turret(Follower follower, HardwareMap hardwareMap) {
        this.turretOrientation = new TurretTracker(hardwareMap, follower);

        // Initialize PID controller
        turretPID = new PIDFController(new PIDFCoefficients(kP, kI, kD, kF));

        turretOrientation.encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretOrientation.encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretOrientation.encoder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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

    private double getTurretDegrees() {
        return turretOrientation.getTurretAngle() * (180.0 / Math.PI);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public void update(double robotHeading) {
        if (!autoAlign) return;

        long currentTime = System.currentTimeMillis();
        if (currentTime - lastUpdateTime < UPDATE_INTERVAL_MS) {
            return;
        }
        lastUpdateTime = currentTime;

        // Calculate desired angle to target
        double targetFieldAngleRad = turretOrientation.calculateDesiredTurretAngle();
        double desiredTurretAngleDeg = Math.toDegrees(targetFieldAngleRad);

        // Get current turret angle
        double currentTurretDeg = getTurretDegrees();

        // Calculate error (shortest path)
        double errorDeg = normalizeAngle(desiredTurretAngleDeg - currentTurretDeg);

        // Stop if within settle zone
        if (Math.abs(errorDeg) <= settleZone) {
            turretOrientation.encoder.setPower(0);
            return;
        }

        // Convert to ticks for PID
        int currentTicks = turretOrientation.encoder.getCurrentPosition();
        int targetTicks = currentTicks + degreesToTicks(errorDeg);

        // Run PID
        turretPID.setTargetPosition(targetTicks);
        turretPID.updatePosition(currentTicks);
        double pidOutput = turretPID.run();

        // Apply power limits
        double power = Range.clip(pidOutput, -maxPower, maxPower);

        // Apply minimum power threshold
        if (Math.abs(power) > 0 && Math.abs(power) < minPower) {
            power = Math.signum(power) * minPower;
        }

        // Set motor power
        turretOrientation.encoder.setPower(power);
    }

    public boolean isOnTarget() {
        double desiredTurretAngleRad = turretOrientation.calculateDesiredTurretAngle();
        double desiredTurretAngleDeg = Math.toDegrees(desiredTurretAngleRad);

        double currentTurretDeg = getTurretDegrees();
        double error = normalizeAngle(desiredTurretAngleDeg - currentTurretDeg);

        return Math.abs(error) <= angleTolerance;
    }
}