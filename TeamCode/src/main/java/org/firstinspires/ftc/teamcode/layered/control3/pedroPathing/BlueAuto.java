package org.firstinspires.ftc.teamcode.layered.control3.pedroPathing; // make sure this aligns with class location

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Blue Auto", group = "Blue")
@Configurable
public class BlueAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(34, 134, Math.toRadians(0));
    private final Pose obelisqueScanPose = new Pose(72, 71, Math.toRadians(-90));
    private final Pose shootPose = new Pose(72, 71, Math.toRadians(-45));
    private final Pose pickup1Pose = new Pose(44, 84, Math.toRadians(0));
    private final Pose pickup1EndPose = new Pose(17, 84, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(58, 60, Math.toRadians(0));
    private final Pose pickup2EndPose = new Pose(17, 60, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(58, 36, Math.toRadians(0));
    private final Pose pickup3EndPose = new Pose(17, 36, Math.toRadians(0));
    private PathChain getObelisqueScan, scorePreload;
    private PathChain goToPickup1, grabPickup1, scorePickup1, goToPickup2, grabPickup2, scorePickup2, goToPickup3, grabPickup3, scorePickup3;

    public void buildPaths() {
        getObelisqueScan = follower.pathBuilder()
                .addPath(new BezierLine(startPose, obelisqueScanPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), obelisqueScanPose.getHeading())
                .build();

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(obelisqueScanPose, shootPose))
                .setLinearHeadingInterpolation(obelisqueScanPose.getHeading(), shootPose.getHeading())
                .build();


        goToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickup1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickup1Pose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup1EndPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1EndPose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1EndPose, shootPose))
                .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), shootPose.getHeading())
                .build();

        goToPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickup2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickup2Pose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup2EndPose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2EndPose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2EndPose, shootPose))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), shootPose.getHeading())
                .build();

        goToPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickup3Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickup3Pose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, pickup3EndPose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickup3EndPose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3EndPose, shootPose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(getObelisqueScan, true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(scorePreload, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(goToPickup1, true);
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

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
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
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}
