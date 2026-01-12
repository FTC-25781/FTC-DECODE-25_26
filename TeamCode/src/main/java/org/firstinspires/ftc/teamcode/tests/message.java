//package org.firstinspires.ftc.teamcode.tests;
//
//import com.pedropathing.control.PIDFCoefficients;
//import com.pedropathing.control.PIDFController;
//import com.pedropathing.follower.Follower;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorImplEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Layers.Physicals.MotorPhys;
//
//public class TurretLogical {
//    MotorPhys turret;
//    double kP = 0.009;
//    double kI = 0;
//    double kD = 0.0003;
//    double kF = 0.000;
//    PIDFController turretPID = new PIDFController(new PIDFCoefficients(kP, kI, kD, kF));
//
//    double direction = 0;
//
//    double difference;
//
//    double headingDisplacement;
//
//    private Telemetry telemetry;
//
//    static final double TICKS_PER_180_DEG = 622;
//    static final double DEGREES_PER_180_TICKS = 180.0;
//
//    static final double TICKS_PER_DEGREE = TICKS_PER_180_DEG / DEGREES_PER_180_TICKS;
//    static final double DEGREES_PER_TICK = DEGREES_PER_180_TICKS / TICKS_PER_180_DEG;
//
//    private boolean autoAlign;
//
//    public TurretLogical(HardwareMap hardwareMap, Telemetry telemetry, String turretName, boolean motorReverse) {
//        this.turret = new MotorPhys(hardwareMap.get(DcMotorEx.class, turretName), motorReverse, false, MotorPhys.ControlMode.POWER);
//        this.telemetry = telemetry;
//    }
//    private int degreesToTicks(double degrees) {
//        return (int) Math.round(degrees * TICKS_PER_DEGREE);
//    }
//
//    public double getTurretDegrees() {
//        return turret.getCurrentValue() * DEGREES_PER_TICK;
//    }
//    public void reset_Init(){
//        turret.motor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
//        turret.motor.setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.FLOAT);
//        turret.motor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
//        turretPID.setTargetPosition(0);
//    }
//    public void startAutoAlign(){
//        autoAlign = true;
//    }
//    public void stopAutoAlign(){
//        autoAlign = false;
//    }
//
//    public void update(double x, double y, double heading, boolean red, double offSet, Telemetry telemetry){
//        turretPID.setCoefficients(new PIDFCoefficients(kP, kI, kD, kF));
//
//
//        int bluetargetAngle = angleToPointDegrees(x, y, 0, 144);
//        int redtargetAngle = angleToPointDegrees(x, y, 144, 144);
//
//        headingDisplacement = offSet - Math.toDegrees(heading); //0 is turret starting displacement
//
//        if(red){
//            difference = redtargetAngle + headingDisplacement;
//        }else{
//            difference = bluetargetAngle + headingDisplacement;
//        }
//
//        if (difference > 180){
//            direction = -360;
//        }else if(difference < -180){
//            direction = 360;
//        }else{
//            direction = 0;
//        }
//
//
//        if(!red && autoAlign){
//            turretPID.setTargetPosition(degreesToTicks(direction + (bluetargetAngle + headingDisplacement)));
//        }
//        else if (autoAlign){
//            turretPID.setTargetPosition(degreesToTicks(direction + (redtargetAngle + headingDisplacement)));
//        }
//        if(autoAlign) {
//            turretPID.updatePosition(turret.getCurrentPosition());
//            turret.motor.setPower(turretPID.run());
//        }else{
//            turret.motor.setPower(0);
//        }
//
//        telemetry.addData("power", turretPID.run());
//        telemetry.addData("target", turretPID.getTargetPosition());
//        telemetry.addData("turret position", turret.getCurrentPosition());
//        telemetry.addData("Error", turretPID.getError());
//    }
///**
//    public void startAutoAlign(){autoAlign = true;}
//    public void stopAutoAlign(){autoAlign = false;}
//
// **/
//
//    public int angleToPointDegrees(double curX, double curY, double targetPointX, double targetPointY){
//        double xDifference = targetPointX - curX;
//        double yDifference = targetPointY - curY; // Difference between the two points
//        //angle formula: arctangent(yDiff / xDiff)
//        double angleRad = (Math.atan2(yDifference, xDifference));
//        //since arctangent gives back in radians, not degrees
//        int angleDeg = (int)(Math.toDegrees(angleRad));
//        //Integer because doesn't really get the exact heading due to sensor or human error
//        return angleDeg;
//    }
//}