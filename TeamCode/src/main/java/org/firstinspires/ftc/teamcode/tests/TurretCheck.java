package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.layeredFinal.logical.TurretTracker;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartLimelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="Turret Tester Checker")
public class TurretCheck extends OpMode {

    Follower follower;
    public TurretTracker turret;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));

        // Initialize turret
        turret = new TurretTracker(hardwareMap,follower);
    }

    @Override
    public void start() {
        follower.startTeleopDrive(false);
    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("Turret local angle", turret.turretLocalAngle());
        telemetry.addData("Turret Global Angle", turret.turretGolbalAngle());
        telemetry.addData("Robot Angle", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}