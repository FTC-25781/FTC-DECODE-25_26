//package org.firstinspires.ftc.teamcode.tests;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.layeredFinal.logical.Turret;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Disabled
//@TeleOp(name="Turret Tester")
//public class TurretTest extends OpMode {
//    Follower follower;
//
//    public Turret turret;
//
//    @Override
//    public void init(){
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(72, 72, Math.toRadians(135)));
//
//        turret = new Turret(hardwareMap, follower);
//    }
//    @Override
//    public void loop(){
//        follower.update();
//        turret.trackGoal();
//
//        if(gamepad1.dpadUpWasPressed()){
//            turret.kP += 0.1;
//        }
//        if(gamepad1.dpadDownWasPressed()) {
//            turret.kP -= 0.1;
//        }
//
//        if (gamepad1.aWasPressed()) {
//            turret.turretOrientation.isRed = true;
//        }
//        if (gamepad1.bWasPressed()) {
//            turret.turretOrientation.isRed = false;
//        }
//        if(gamepad1.xWasPressed()){
//            turret.turretOrientation.resetEncoder();
//        }
//
//        // Add this to your telemetry
//        telemetry.addData("Encoder Ticks", turret.turretOrientation.encoder.getCurrentPosition());
//        telemetry.addData("Turret On Target", turret.isOnTarget() ? "On target": "Tracking");
//        telemetry.addData("Turret Angle", turret.turretOrientation.getTurretAngle());
//        telemetry.addData("Desired turret angle", turret.turretOrientation.calculateDesiredTurretAngle());
//        telemetry.addData("Error", turret.turretOrientation.calculateError());
//        telemetry.addData("kP", turret.kP);
//        telemetry.addData("X-Pos", follower.getPose().getX());
//        telemetry.addData("Y-Pos", follower.getPose().getY());
//        telemetry.addData("Heading", follower.getPose().getHeading());
//        telemetry.addData("Ticks for 360°", turret.turretMotor.getCurrentPosition());
//        telemetry.update();
//    }
//}
//