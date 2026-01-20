package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {

    public TurretTracker turretOrientation;
    public PIDFController turretPID;
    public boolean autoAlign = false;
    public boolean redAlliance = true;
    //public static double RED_OFFSET = 15.4948;
    //public static double BLUE_OFFSET = -14.5106;

    public double kP = 0.081;
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
        turretPID.setTargetPosition(0);
    }

    public void setAlliance(boolean red) {
        this.redAlliance = red;
        this.turretOrientation.isRed = red;
    }
    public void startAutoAlign() {
        autoAlign = true;
    }
    public void stopAutoAlign(){
        autoAlign = false;
    }
    private double normalizeAngle(double angle) {
        return Math.IEEEremainder(angle, 360.0);
    }

    public void update() {
        if (!autoAlign){
            turretOrientation.encoder.setPower(0);
            return;
        }
        turretPID.setP(kP);
        turretPID.setI(kI);
        turretPID.setD(kD);
        turretPID.setF(kF);
        double worldTarget = turretOrientation.getAngleToGoal();
        double robotHeading = Math.toDegrees(turretOrientation.follower.getHeading());
        //double initialAngle = turretOrientation.turretGolbalAngle();
        // eqn: for red at (72, 72) 45 - 90 = -45 deg of turret rotation ( -45 deg -> ticks = -45 * TICKS_PER_DEGREE)
        // eqn: for blue at (72, 72) 135 - 90 = 45 deg of turret rotation (45 deg -> ticks = 45 * TICKS_PER_DEGREE)

        double necessaryRotation = worldTarget - robotHeading;

        while (necessaryRotation > 180) necessaryRotation -= 360;
        while (necessaryRotation <= -180) necessaryRotation += 360;

        necessaryRotation *= TICKS_PER_DEGREE;
        turretPID.updatePosition(turretOrientation.encoder.getCurrentPosition());
        turretPID.setTargetPosition((int)(Math.round(necessaryRotation)));

        double currentTicks = turretOrientation.encoder.getCurrentPosition();
        turretPID.updateError(necessaryRotation - currentTicks);

        turretOrientation.encoder.setPower(turretPID.run());
    }
    public boolean isOnTarget() {
        double desiredTurretAngleDeg = turretOrientation.calculateDesiredTurretAngle();
        double currentTurretDeg = turretOrientation.turretLocalAngle();
        double error = normalizeAngle(desiredTurretAngleDeg - currentTurretDeg);
        return Math.abs(error) <= angleTolerance;
    }
}