package org.firstinspires.ftc.teamcode.layered.logical2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.layered.control3.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.layered.physical1.FlywheelMotor;
import org.firstinspires.ftc.teamcode.layered.physical1.ServoForTransfer;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


@TeleOp(name="Angular PID")
public class PIDShooter extends LinearOpMode {
    public ServoForTransfer servo_t;
    public FlywheelMotor flywheel;
    public GetDistanceToGoal distanceCalculation;
    public Pose startingPose = new Pose(72, 72, Math.toRadians(135));
    public Follower follower;
    public long shooter_timer = 0;
    public boolean isShooting = false;

    @Override
    public void runOpMode(){
        initHardware();

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        while(opModeIsActive()){
            follower.update();

            flywheel.updatePIDFCoefficients();

            double distance = distanceCalculation.getDistanceToGoal();
            double targetRPM = flywheel.targetRPM(distance);
            flywheel.setRPM(targetRPM);



            /*
            if(flywheel.isShooterReady()){
                servo_t.moveUp();
                shooter_timer = System.currentTimeMillis();
                isShooting = true;
            }
            if(isShooting && (System.currentTimeMillis() - shooter_timer) > 500){
                servo_t.moveDown();
                isShooting = false;
            }
             */

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target RPM", targetRPM);
            packet.put("Current RPM", flywheel.getCurrentRPM());
            packet.put("Distance", distance);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", flywheel.getCurrentRPM());
            telemetry.addData("Target Velocity (rad/s)", flywheel.flywheelShooter.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("Motor Power", flywheel.flywheelShooter.getPower());
            telemetry.addData("Motor Mode", flywheel.flywheelShooter.getMode());
            telemetry.addData("kP", FlywheelMotor.kP);
            telemetry.addData("kI", FlywheelMotor.kI);
            telemetry.addData("kD", FlywheelMotor.kD);
            telemetry.addData("kF", FlywheelMotor.kF);
            telemetry.update();
        }
    }
    public void initHardware(){
        servo_t = new ServoForTransfer(hardwareMap);
        flywheel = new FlywheelMotor(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        distanceCalculation = new GetDistanceToGoal(follower, true);
    }
}