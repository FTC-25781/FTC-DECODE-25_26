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

    /*
    static final double TICKS_PER_180_DEG = 182;
    static final double DEGREES_PER_180_TICKS = 180.0;
     */
    static final double TICKS_PER_DEGREE = 182 / 180.0;

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

    public void update() {
        if (!autoAlign){
            turretOrientation.encoder.setPower(0);
            return;
        }
        //double initialAngle = turretOrientation.turretGolbalAngle();
        // eqn: for red at (72, 72) 45 - 90 = -45 deg of turret rotation ( -45 deg -> ticks = -45 * TICKS_PER_DEGREE)
        // eqn: for blue at (72, 72) 135 - 90 = 45 deg of turret rotation (45 deg -> ticks = 45 * TICKS_PER_DEGREE)

        double necessaryRotation = turretOrientation.getAngleToGoal() -
                Math.toDegrees(turretOrientation.follower.getHeading());

        while (necessaryRotation > 180) necessaryRotation -= 360;
        while (necessaryRotation <= -180) necessaryRotation += 360;

        turretPID.updatePosition(turretOrientation.encoder.getCurrentPosition());
        turretPID.setTargetPosition(necessaryRotation* TICKS_PER_DEGREE);
        // turretPID.updateError(necessaryRotation* TICKS_PER_DEGREE - turretOrientation.encoder.getCurrentPosition());

        turretOrientation.encoder.setPower(turretPID.run());
    }
}