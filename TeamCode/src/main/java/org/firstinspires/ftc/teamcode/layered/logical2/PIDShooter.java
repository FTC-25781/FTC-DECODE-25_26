package org.firstinspires.ftc.teamcode.layered.logical2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.layered.control3.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.layered.physical1.FlywheelMotor;
import org.firstinspires.ftc.teamcode.layered.physical1.ServoForTransfer;


@TeleOp(name="Angular PID")
public class PIDShooter extends LinearOpMode {
    public ServoForTransfer servo_t;
    public FlywheelMotor flywheel;
    public GetDistanceToGoal distanceCalculation;
    public Pose startingPose = new Pose(72, 72, Math.toRadians(45));
    public Follower follower;
    public boolean isRedAlliance = true;
    public long shooter_timer = 0;
    public boolean isShooting = false;

    @Override
    public void runOpMode(){
        initHardware();
        waitForStart();

        while(opModeIsActive()){
            follower.update();

            double distance = distanceCalculation.getDistanceToGoal();
            double targetRPM = flywheel.targetRPM(distance);
            flywheel.setRPM(targetRPM);

            if(flywheel.isShooterReady()){
                servo_t.moveUp();
                shooter_timer = System.currentTimeMillis();
                isShooting = true;
            }
            if(isShooting && (System.currentTimeMillis() - shooter_timer) > 500){
                servo_t.moveDown();
                isShooting = false;
            }
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Shooting?", isShooting);
            telemetry.update();
        }
    }

    public void initHardware(){
        servo_t = new ServoForTransfer(hardwareMap);
        flywheel = new FlywheelMotor(hardwareMap, telemetry);
        distanceCalculation = new GetDistanceToGoal(follower, true);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
    }
}