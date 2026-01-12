package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretTracker {

    public DcMotor encoder;
    public static final double TICKS_PER_REV = 364;
    Follower follower;
    public GoBildaPinpointDriver pinpoint;
    public double blueX = 12;
    public double blueY = 132;
    public double redX = 132;
    public double redY = 132;
    public boolean isRed = false;

    public TurretTracker(HardwareMap hardwareMap, Follower follower){
        encoder = hardwareMap.get(DcMotor.class, "tmot");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setDirection(DcMotorSimple.Direction.REVERSE);
        this.follower = follower;
    }

    /**
     * This code converts the ticks that the encoder reads into
     * radians to figure out the angle of the turret relative to the robot
     */
    public double getTurretAngle(){
        double ticks = encoder.getCurrentPosition();
        double angle = (ticks / TICKS_PER_REV) * (2 * Math.PI);
        return normalizeAngle(angle);
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
        encoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Calculates the angle to the goal (field angle to goal) and returns that angle in
     * radians
     */
    public double getAngleToGoal(){
         follower.update();
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double angle;
        if(isRed){
            angle =  Math.atan2(redY - robotY, redX - robotX);
        }
        else{
            angle =  Math.atan2(blueY - robotY, blueX - robotX);
        }
        return angle;
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
