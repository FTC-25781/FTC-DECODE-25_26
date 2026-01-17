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

    // PID Constants
    public final double kP = 1.512; // 2.78
    public final double kI = 0.0;
    public final double kD = 0.0003; // 0.13
    public final double kF = 0.008;

    // Turret target coordinates
    private double redGoalX = 144;
    private double redGoalY = 144;
    private double blueGoalX = 0;
    private double blueGoalY = 144;

    // Angle tolerance (2 degrees)
    public double angleTolerance = 2; // degrees

    // Encoder conversion
    static final double TICKS_PER_180_DEG = 622;
    static final double DEGREES_PER_180_TICKS = 180.0;
    static final double TICKS_PER_DEGREE = TICKS_PER_180_DEG / DEGREES_PER_180_TICKS;

    public Turret(Follower follower, HardwareMap hardwareMap) {
        this.turretOrientation = new TurretTracker(hardwareMap, follower);
        turretPID = new PIDFController(new PIDFCoefficients(kP, kI, kD, kF));

        turretOrientation.encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretOrientation.encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretOrientation.encoder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setAlliance(boolean red) {
        this.redAlliance = red;
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
        return turretOrientation.getTurretAngle() * (180 / Math.PI);
    }

    public void update(double robotHeading) {
        if (!autoAlign) return;

        // Robot position
        double robotX = turretOrientation.follower.getPose().getX();
        double robotY = turretOrientation.follower.getPose().getY();

        // Select goal
        double targetX = redAlliance ? redGoalX : blueGoalX;
        double targetY = redAlliance ? redGoalY : blueGoalY;

        // Calculate angle to target in degrees
        double targetFieldAngleRad = Math.atan2(targetY - robotY, targetX - robotX);
        double desiredTurretAngleRad = targetFieldAngleRad - robotHeading;
        double desiredTurretAngleDeg = Math.toDegrees(desiredTurretAngleRad);

        // Current turret angle in degrees
        double currentTurretDeg = getTurretDegrees();

        // Calculate shortest path difference
        double difference = desiredTurretAngleDeg - currentTurretDeg;
        double direction = 0;
        if (difference > 180) direction = -360;
        else if (difference < -180) direction = 360;

        // PID target in ticks
        int targetTicks = degreesToTicks(currentTurretDeg + difference + direction);

        // Update PID
        turretPID.setCoefficients(new PIDFCoefficients(kP, kI, kD, kF));
        turretPID.setTargetPosition(targetTicks);
        turretPID.updatePosition(turretOrientation.encoder.getCurrentPosition());
        double output = turretPID.run();

        // Apply clipped power
        output = Range.clip(output, -1, 1);

        if (Math.abs(difference) <= angleTolerance) {
            turretOrientation.encoder.setPower(0);
        } else {
            turretOrientation.encoder.setPower(output);
        }
    }

    public boolean isOnTarget() {
        double robotX = turretOrientation.follower.getPose().getX();
        double robotY = turretOrientation.follower.getPose().getY();
        double robotHeading = turretOrientation.follower.getPose().getHeading();

        double targetX = redAlliance ? redGoalX : blueGoalX;
        double targetY = redAlliance ? redGoalY : blueGoalY;

        double targetFieldAngleRad = Math.atan2(targetY - robotY, targetX - robotX);
        double desiredTurretAngleRad = targetFieldAngleRad - robotHeading;
        double desiredTurretAngleDeg = Math.toDegrees(desiredTurretAngleRad);

        double currentTurretDeg = getTurretDegrees();
        double error = desiredTurretAngleDeg - currentTurretDeg;

        if (error > 180) error -= 360;
        else if (error < -180) error += 360;

        return Math.abs(error) <= angleTolerance;
    }
}
