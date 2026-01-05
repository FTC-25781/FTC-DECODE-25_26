package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.layeredFinal.control.Intake;

// Shreesh Approved Auto
@Autonomous(name = "Red Auto Bottom", group = "Red")
public class RedAutoBottom extends OpMode {
    private Intake intake;


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(94, 8, Math.toRadians(0));
    private final Pose scanAndShootPreload = new Pose(94, 9, Math.toRadians(0));
    private final Pose pickup1EndPose = new Pose(131,37, Math.toRadians(0));
    private final Pose pickup1ControlPose = new Pose(72, 38, Math.toRadians(0));
    private final Pose humanPlayerPose = new Pose(135, 8, Math.toRadians(0));

    private Path scanAndShoot;
    private PathChain goToPickup1, goToScore, goToHumanPlayerZone, goToScore2;

    public void buildPaths() {
        scanAndShoot = new Path(new BezierLine(startPose, scanAndShootPreload));
        scanAndShoot.setConstantHeadingInterpolation(scanAndShootPreload.getHeading());

        goToPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, pickup1ControlPose, pickup1EndPose))
                .setConstantHeadingInterpolation(pickup1EndPose.getHeading())
                .build();

        goToScore = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1EndPose, pickup1ControlPose, startPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        goToHumanPlayerZone = follower.pathBuilder()
                .addPath(new BezierLine(startPose, humanPlayerPose))
                .setConstantHeadingInterpolation(humanPlayerPose.getHeading())
                .build();

        goToScore2 = follower.pathBuilder()
                .addPath(new BezierLine(humanPlayerPose, startPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scanAndShoot);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    /* Score Preload */
                    intake.forward();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(goToPickup1, true);
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(goToScore, true);
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(goToHumanPlayerZone, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    intake.stopped();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(goToScore2, true);
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
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

    /**
     * This method is called once at the init of the OpMode.
     **/
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

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}
