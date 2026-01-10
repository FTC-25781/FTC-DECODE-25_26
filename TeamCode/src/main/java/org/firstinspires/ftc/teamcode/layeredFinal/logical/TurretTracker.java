package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// TODO: Test please

public class TurretTracker {

    public DcMotor encoder;
    public static final double TICKS_PER_REV = 8192;
    Follower follower;
    public double goalX = 12;
    public double goalY = 132;

    public TurretTracker(HardwareMap hardwareMap, Follower follower){
        encoder = hardwareMap.get(DcMotor.class, "turretEncoder");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.follower = follower;
    }

    /**
     * This code converts the ticks that the encoder reads into
     * radians to figure out the angle of the turret relative to the robot
     */
    public double getTurretAngle(){
        double ticks = encoder.getCurrentPosition();
        return (ticks / TICKS_PER_REV) * (2 * Math.PI);
    }

    /**
     * Used for telemetry later on
     */
    public double turretAngleDegrees(){
        return Math.toDegrees(getTurretAngle());
    }
    public void resetEncoder() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Calculates the angle to the goal (field angle to goal) and returns that angle in
     * radians
     */
    public double getAngleToGoal(){
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        return Math.atan2(goalY - robotY, goalX - robotX);
    }

    /**
     * Subtracts the field angle to goal and the robot orientation to find desired turret angle
     *     Gets turret angle that the servo needs to rotate to to face the goal and
     *     puts that angle in a -180 degree to 180 degree range instead of -360 to 360 degree range
     */
    public double calculateDesiredTurretAngle(){
        double angleToGoal = getAngleToGoal();
        double robotHeading = follower.getPose().getHeading();
        return normalizeAngle(angleToGoal - robotHeading);
    }

    /**
     *This method is for calculating the error between the desired angle and
     *     turret angle and we put the angle in a -180 to 180 range so that it takes the shortest
     *     path
     */
    public double calculateError() {
        double desiredAngle = calculateDesiredTurretAngle();
        double currentAngle = getTurretAngle();
        return normalizeAngle(desiredAngle - currentAngle);
    }

    /**
     * Since 2 * pi is one full revolution of a circle, if the angle in radians is greater than
     * pi, we subtract one full rotation of the angle to put it in a -180 to 180 range
     */
    public double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
