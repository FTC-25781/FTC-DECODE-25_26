package org.firstinspires.ftc.teamcode.layered.control3.pedroPathing; // make sure this aligns with class location

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.layered.logical2.Shooter;
import org.firstinspires.ftc.teamcode.layered.physical1.EncoderForIntake;
import org.firstinspires.ftc.teamcode.layered.physical1.ServoForSorter;
import org.firstinspires.ftc.teamcode.layered.physical1.ServoForTransfer;

@Autonomous(name = "Red Auto", group = "Blue")
@Configurable
public class RedAuto extends OpMode {
//    private Shooter shooter;
    private ServoForSorter servo;
    private ServoForTransfer servo_t;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(110, 133, Math.toRadians(180));
    private final Pose scanPose = new Pose(72, 72, Math.toRadians(90));
    private final Pose shootPose = new Pose(80, 80, Math.toRadians(45));
    private PathChain getScan, shootPreload;

    public void buildPaths() {
        getScan = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scanPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading())
                .build();

        shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(scanPose, shootPose))
                .setLinearHeadingInterpolation(scanPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(getScan, true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
//                    if (actionTimer.getElapsedTime() == 1.0) {
//
//                    }
//
//                    if (actionTimer.getElapsedTime() == 0.2) {
//                        servo_t.moveUp();
//                    }

                    follower.followPath(shootPreload, true);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

//        servo_t.update();
//        servo.update(telemetry);
//        shooter.update(telemetry);

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
//        servo = new ServoForSorter(hardwareMap);
//        servo_t = new ServoForTransfer(hardwareMap);
//        shooter = new Shooter(hardwareMap);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
//        actionTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}
