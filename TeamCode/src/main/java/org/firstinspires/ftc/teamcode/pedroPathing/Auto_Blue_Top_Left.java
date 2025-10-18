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

@Autonomous(name = "Example Auto", group = "Examples")
public class Auto_Blue_Top_Left extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private final Pose startPose = new Pose(33.600, 135.400, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose obelisqueScanPose = new Pose(49.1, 111.9, Math.toRadians(65)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose shootPose = new Pose(49.1, 111.9, Math.toRadians(135));
    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1ControlPose = new Pose(67.7, 82.7, Math.toRadians(180));
    private final Pose pickup1EndPose = new Pose(14.7, 84.1, Math.toRadians(180));
    private final Pose pickup1ShootingControlPose = new Pose(43.2, 96.4, Math.toRadians(135));
    private final Pose pickup2Pose = new Pose(49.1, 58.6, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup2ControlPose = new Pose(94.7, 56.1, Math.toRadians(180));
    private final Pose pickup2EndPose = new Pose(15.7, 58.9, Math.toRadians(180));
    private final Pose pickup2ShootingControlPose = new Pose(59.6, 72.4, Math.toRadians(135));
    private final Pose pickup3Pose = new Pose(49.8, 35.6, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup3ControlPose = new Pose(94.9, 33.9, Math.toRadians(180));
    private final Pose pickup3EndPose = new Pose(15, 35.6, Math.toRadians(180));
    private final Pose pickup3ShootingControlPose = new Pose(46.9, 54.2, Math.toRadians(135));
    private PathChain getObeliskScan, scorePreload;
    private PathChain goToPickup1, grabPickup1, scorePickup1, goToPickup2, grabPickup2, scorePickup2, goToPickup3, grabPickup3, scorePickup3;

    public void buildPaths() {
//        getObeliskScan = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, obelisqueScanPose))
//                .build();
//
//        getObeliskScan = new Path(new BezierLine(startPose, obelisqueScanPose));
//        getObeliskScan.setLinearHeadingInterpolation(startPose.getHeading(), obelisqueScanPose.getHeading());
//
//        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        scorePreload = new Path(new BezierLine(obelisqueScanPose, shootPose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        goToPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, pickup1ControlPose, pickup1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup1EndPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1EndPose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1EndPose, pickup1ShootingControlPose, shootPose))
                .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), shootPose.getHeading())
                .build();

        goToPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, pickup2ControlPose, pickup2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup2EndPose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2EndPose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2EndPose, pickup2ShootingControlPose, shootPose))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), shootPose.getHeading())
                .build();

        goToPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, pickup3ControlPose, pickup3Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, pickup3EndPose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickup3EndPose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3EndPose, pickup3ShootingControlPose, shootPose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(getObeliskScan);
                setPathState(1);
                break;
            case 1:
                follower.followPath(scorePreload);
                setPathState(2);
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(goToPickup1, false);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1, false);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1, false);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(goToPickup2, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabPickup2, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(8);
                }
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(goToPickup3, true);
                    setPathState(9);
                }
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(10);
                }
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(11);
                }
            case 90:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
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