package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="DriveTrain for Arhan")
public class DriveTrainPracticeForArhan extends OpMode {

    Follower follower;
    @Override
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 135));
    }
    @Override
    public void loop(){
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
    }
}
