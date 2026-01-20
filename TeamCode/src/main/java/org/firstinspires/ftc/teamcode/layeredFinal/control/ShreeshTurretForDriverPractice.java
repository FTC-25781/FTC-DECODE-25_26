package org.firstinspires.ftc.teamcode.layeredFinal.control;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Layers.Physicals.MotorPhys;

public class ShreeshTurretForDriverPractice {
    DcMotorEx turret;
    double kP = 0.091;
    double kI = 0;
    double kD = 0.002;
    double kF = 0.000;
    PIDFController turretPID = new PIDFController(new PIDFCoefficients(kP, kI, kD, kF));

    double direction = 0;

    double difference;

    double headingDisplacement;

    private Telemetry telemetry;

    static final double TICKS_PER_180_DEG = 182;
    static final double DEGREES_PER_180_TICKS = 180.0;

    static final double TICKS_PER_DEGREE = TICKS_PER_180_DEG / DEGREES_PER_180_TICKS;
    static final double DEGREES_PER_TICK = DEGREES_PER_180_TICKS / TICKS_PER_180_DEG;

    private boolean autoAlign;

    public ShreeshTurretForDriverPractice(HardwareMap hardwareMap, Telemetry telemetry) {
        this.turret = hardwareMap.get(DcMotorEx.class, "tmot");;
        this.telemetry = telemetry;

        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        turret.setDirection(DcMotorEx.Direction.REVERSE);
    }
    private int degreesToTicks(double degrees) {
        return (int) Math.round(degrees * TICKS_PER_DEGREE);
    }

    public double getTurretDegrees() {
        return turret.getCurrentPosition() * DEGREES_PER_TICK;
    }
    public void reset_Init(){
        turret.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.FLOAT);
        turret.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
        turretPID.setTargetPosition(0);
    }
    public void startAutoAlign(){
        autoAlign = true;
    }
    public void stopAutoAlign(){
        autoAlign = false;
    }

    public void update(double x, double y, double heading, boolean red, double turretStartingOffset, Telemetry telemetry, double turretOffset){
        turretPID.setCoefficients(new PIDFCoefficients(kP, kI, kD, kF));
        boolean robotOutOfPosition = false;

        int bluetargetAngle = angleToPointDegrees(x, y, 0, 140);
        int redtargetAngle = angleToPointDegrees(x, y, 140, 140);

        headingDisplacement = turretStartingOffset - Math.toDegrees(heading); // 0 is turret starting displacement

        if(red){
            difference = redtargetAngle + headingDisplacement + turretOffset;
        }else{
            difference = bluetargetAngle + headingDisplacement + turretOffset;
        }

        if (difference > 180){
            direction = -360;
        }else if(difference < -180){
            direction = 360;
        }else{
            direction = 0;
        }

        if (Math.toDegrees(heading) < -45 || Math.toDegrees(heading) > 135){ robotOutOfPosition = true;}
        else {robotOutOfPosition = false;}

        if(!red && autoAlign){
            turretPID.setTargetPosition(degreesToTicks(direction + (bluetargetAngle + headingDisplacement)+turretOffset));
        }
        else if (autoAlign && !robotOutOfPosition){
            turretPID.setTargetPosition(degreesToTicks(direction + (redtargetAngle + headingDisplacement)+turretOffset));
        }
        if(autoAlign) {
            turretPID.updatePosition(turret.getCurrentPosition());
            turret.setPower(turretPID.run());
        }else{
            turret.setPower(0);
        }

        telemetry.addData("power", turretPID.run());
        telemetry.addData("target", turretPID.getTargetPosition());
        telemetry.addData("turret position", turret.getCurrentPosition());
        telemetry.addData("turret offset angle", turretOffset);
        telemetry.addData("Error", turretPID.getError());
    }
    /**
     public void startAutoAlign(){autoAlign = true;}
     public void stopAutoAlign(){autoAlign = false;}

     **/

    public int angleToPointDegrees(double curX, double curY, double targetPointX, double targetPointY){
        double xDifference = targetPointX - curX;
        double yDifference = targetPointY - curY; // Difference between the two points
        //angle formula: arctangent(yDiff / xDiff)
        double angleRad = (Math.atan2(yDifference, xDifference));
        //since arctangent gives back in radians, not degrees
        int angleDeg = (int)(Math.toDegrees(angleRad));
        //Integer because doesn't really get the exact heading due to sensor or human error
        return angleDeg;
    }
}
