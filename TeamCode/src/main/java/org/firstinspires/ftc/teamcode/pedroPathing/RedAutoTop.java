package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.layeredFinal.control.Intake;

@Autonomous(name = "Red Auto Top", group = "Red")
public class RedAutoTop extends OpMode {
    private Intake intake;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

   private final Pose startPose = new Pose(123.5, 123.5, Math.toRadians(45));
    public PathChain ScanAndShootPreload;
    public PathChain GoToPickup1;
    public PathChain ShootPreload1;
    public PathChain GoToPick2;
    public PathChain OpenTheGate;
    public PathChain ShootPick2;

    public void buildPaths() {
        ScanAndShootPreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(123.500, 123.500),
                        new Pose(96.000, 96.000)
                        )).setConstantHeadingInterpolation(Math.toRadians(45))
                .build();

        GoToPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                                new Pose(96.000, 96.000),
                                new Pose(63.000, 83.000),
                                new Pose(126.000, 84.000)
                        )).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        ShootPreload1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                                new Pose(126.000, 84.000),
                                new Pose(93.000, 80.000),
                                new Pose(96.000, 96.000)
                        )).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        GoToPick2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                                new Pose(96.000, 96.000),
                                new Pose(57.000, 55.000),
                                new Pose(126.000, 60.000)
                        )).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        OpenTheGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                                new Pose(126.000, 60.000),
                                new Pose(115.000, 70.000),
                                new Pose(129.000, 68.000)
                        )).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        ShootPick2 = follower.pathBuilder()
                .addPath(new BezierLine(
                                new Pose(129.000, 68.000),
                                new Pose(72.000, 72.000)
                        )).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(ScanAndShootPreload);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    /* Score Preload */
                    intake.forward();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(GoToPickup1, true);
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(ShootPreload1, true);
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(GoToPick2, true);
                    pathTimer.resetTimer();
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    /* Grab Sample */
                    intake.stopped();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(OpenTheGate, true);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    /* Grab Sample */
                    intake.stopped();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(ShootPick2, true);
                    pathTimer.resetTimer();
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
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        intake = new Intake(hardwareMap);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}
