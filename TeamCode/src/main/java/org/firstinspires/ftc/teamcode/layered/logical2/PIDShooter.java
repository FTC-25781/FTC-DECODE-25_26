package org.firstinspires.ftc.teamcode.layered.logical2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.layered.control3.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.layered.physical1.ServoForTransfer;


@TeleOp(name="Angular PID")
public class PIDShooter extends LinearOpMode {
    public DcMotorEx flywheelShooter;
    public ServoForTransfer servo_t;
    public Pose startingPose = new Pose(72, 72, Math.toRadians(45));
    public Follower follower;
    public static final double TICKS_PER_REV = 28.0;
    public static final double RPM_TOLERANCE = 50.0;
    public boolean isRedAlliance = true;
    public long shooter_timer = 0;
    public boolean isShooting = false;

    @Override
    public void runOpMode(){
        initHardware();

        flywheelShooter.setVelocityPIDFCoefficients(9.25, 2.2, 7, 0);

        waitForStart();

        while(opModeIsActive()){

            follower.update();

            double distance = getDistanceToGoal();
            double targetRPM = targetRPM(distance);
            double targetTicksPerSec = targetRPM * TICKS_PER_REV / 60.0;

            double radPerSec = flywheelShooter.getVelocity(AngleUnit.RADIANS);
            double actualRPM = radPerSec * 60 / (2 * Math.PI);
            boolean shooterReady = isShooterReady(targetRPM, actualRPM);

            flywheelShooter.setVelocity(targetTicksPerSec);

            if(shooterReady && !isShooting){
                servo_t.moveUp();
                shooter_timer = System.currentTimeMillis();
                isShooting = true;
            }
            if(isShooting && (System.currentTimeMillis() - shooter_timer) > 500){
                servo_t.moveDown();
                isShooting = false;
            }
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual RPM", actualRPM);
            telemetry.addData("Shooter Ready", shooterReady);
            telemetry.addData("Shooting?", isShooting);
            telemetry.update();
        }
    }

    public void initHardware(){
        flywheelShooter = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        servo_t = new ServoForTransfer(hardwareMap);
        flywheelShooter.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheelShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
    }
    public double getDistanceToGoal(){
        Pose robotPose = follower.getPose();
        if(gamepad1.a){
            isRedAlliance = false;
            if(gamepad1.b){
                isRedAlliance = true;
            }
        }
        double goalX = isRedAlliance ? 132 : 12;
        double goalY = 132;

        double dx = goalX - robotPose.getX();
        double dy = goalY - robotPose.getY();
        double distanceToGoal = Math.sqrt(dx * dx + dy * dy);

        return distanceToGoal;
    }
    public double targetRPM(double distance){
        double x = distance;

        final double C4 = 7.13229E-9;
        final double C3 = -(0.00000244252);
        final double C2 = 0.000297598;
        final double C1 = -0.0129613;
        final double C0 = 0.819973;

        double targetRPM1 = (C4 * Math.pow(x, 4)) + (C3 * Math.pow(x, 3)) + (C2 * Math.pow(x, 2)) + (C1 * x) + C0;
        targetRPM1 = Range.clip(targetRPM1 * 6000, 0, 6000);
        return targetRPM1;
    }
    public boolean isShooterReady(double target, double actual) {
        return Math.abs(target - actual) < RPM_TOLERANCE;
    }
}