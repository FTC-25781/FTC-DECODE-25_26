package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Blue Auto", group = "Blue")
public class BlueAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(25, 137, Math.toRadians(0));
    private final Pose scanPose = new Pose(42, 101, Math.toRadians(65));
    private final Pose scorePose = new Pose(42, 102, Math.toRadians(135));

    private final Pose pickup1Pose = new Pose(42, 84.2, Math.toRadians(180));
    private final Pose pickup1EndPose = new Pose(15, 84.2, Math.toRadians(180));

    private final Pose pickup2Pose = new Pose(42, 60, Math.toRadians(180));
    private final Pose pickup2EndPose = new Pose(15, 60, Math.toRadians(180));

    private final Pose pickup3Pose = new Pose(42, 35, Math.toRadians(180));
    private final Pose pickup3EndPose = new Pose(15, 35, Math.toRadians(180));

    private Path scan;
    private Path scorePreload;
    private PathChain goToPickup1, grabPickup1, scorePickup1;
    private PathChain goToPickup2, grabPickup2, scorePickup2;
    private PathChain goToPickup3, grabPickup3, scorePickup3;

    public void buildPaths() {

        scan = new Path(new BezierLine(startPose, scanPose));
        scan.setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading());

        scorePreload = new Path(new BezierLine(scanPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(scanPose.getHeading(), scorePose.getHeading());

        goToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup1EndPose))
                .setConstantHeadingInterpolation(pickup1EndPose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1EndPose, scorePose))
                .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), scorePose.getHeading())
                .build();

        goToPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup2EndPose))
                .setConstantHeadingInterpolation(pickup2EndPose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2EndPose, scorePose))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), scorePose.getHeading())
                .build();

        goToPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, pickup3EndPose))
                .setConstantHeadingInterpolation(pickup3EndPose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3EndPose, scorePose))
                .setLinearHeadingInterpolation(pickup3EndPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(scan);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1.0) {
                    follower.followPath(scorePreload, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1.0) {
                    follower.followPath(goToPickup1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1.0) {
                    follower.followPath(grabPickup1, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1.0) {
                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1.0) {
                    follower.followPath(goToPickup2, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1.0) {
                    follower.followPath(grabPickup2, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1.0) {
                    follower.followPath(scorePickup2, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1.0) {
                    follower.followPath(goToPickup3, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1.0) {
                    follower.followPath(grabPickup3, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1.0) {
                    follower.followPath(scorePickup3, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1.0) {
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

        telemetry.addData("Path State", pathState);
        telemetry.addData("Timer", pathTimer.getElapsedTime());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
