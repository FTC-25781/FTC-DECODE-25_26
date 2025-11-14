//package org.firstinspires.ftc.teamcode.layered.control3.pedroPathing; // make sure this aligns with class location
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//@Autonomous(name = "Blue Auto", group = "Blue")
//@Configurable
//public class BlueAuto extends OpMode {
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//    private int pathState;
//
//    private final Pose startPose = new Pose(34, 134, Math.toRadians(0));
//    private final Pose shootPose = new Pose(72, 71, Math.toRadians(-45));
//    private final Pose humanPose = new Pose(72, 71, Math.toRadians(-90));
//    private PathChain shootPreload, goToHumanZone;
//
//    public void buildPaths() {
//        getObelisqueScan = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, obelisqueScanPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), obelisqueScanPose.getHeading())
//                .build();
//
//        shootPreload = follower.pathBuilder()
//                .addPath(new BezierLine(obelisqueScanPose, shootPose))
//                .setLinearHeadingInterpolation(obelisqueScanPose.getHeading(), shootPose.getHeading())
//                .build();
//
//
//        goToHumanZone = follower.pathBuilder()
//                .addPath(new BezierLine(shootPose, pickup1Pose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), pickup1Pose.getHeading())
//                .build();
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                follower.followPath(getObelisqueScan, true);
//                setPathState(1);
//                break;
//            case 1:
//                if(!follower.isBusy()) {
//                    follower.followPath(scorePreload, true);
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                if (!follower.isBusy()) {
//                    follower.followPath(goToPickup1, true);
//                    setPathState(-1);
//                }
//                break;
//        }
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        autonomousPathUpdate();
//
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.update();
//    }
//
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);
//
//    }
//
//    /** This method is called continuously after Init while waiting for "play". **/
//    @Override
//    public void init_loop() {}
//
//    /** This method is called once at the start of the OpMode.
//     * It runs all the setup actions, including building paths and starting the path system **/
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//
//    /** We do not use this because everything should automatically disable **/
//    @Override
//    public void stop() {}
//}
