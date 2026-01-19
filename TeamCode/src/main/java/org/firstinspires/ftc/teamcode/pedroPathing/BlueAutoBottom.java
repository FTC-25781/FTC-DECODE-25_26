package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.layeredFinal.control.Intake;
import org.firstinspires.ftc.teamcode.layeredFinal.control.Transfer;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Flywheel;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Limelight;

@Autonomous(name = "Blue Auto Bottom", group = "Blue")
public class BlueAutoBottom extends OpMode {
    private Intake intake;
    private Transfer transfer;
    private Limelight limelight;
    private Flywheel flywheel;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private boolean timerReset = false;
    private boolean reset = false;

    private Pose startPose = new Pose(94, 8, Math.toRadians(0));
    private Pose scanAndShootPreload = new Pose(94, 9, Math.toRadians(0));
    private Pose pickup1EndPose = new Pose(131,38.5, Math.toRadians(0));
    private Pose pickup1ControlPose = new Pose(72, 38, Math.toRadians(0));
    private Pose humanPlayerPose = new Pose(135, 8, Math.toRadians(0));

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
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1) {
                    limelight.getIDAndLog(limelight.getID());
                    transfer.id = limelight.getLastLoggedID();

                    flywheel.setVelForFarTip();

                    if(!timerReset) {
                        timerReset = true;
                        pathTimer.resetTimer();
                    }

                    if (transfer.currentState == Transfer.State.IDLE &&
                            pathTimer.getElapsedTimeSeconds() > 2.8) {
                        transfer.startKickSequenceRandomly();
                    }

                    if (transfer.currentState == Transfer.State.DONE) {
                        if (!reset) {
                            resetEverything();
                        }

                        intake.forward();

                        follower.followPath(goToPickup1,0.65, true);
                        pathTimer.resetTimer();
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    reset = false;
                    follower.followPath(goToScore, true);
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    intake.reverse();
                    flywheel.setVelForFarTip();

                    if(!timerReset) {
                        timerReset = true;
                        pathTimer.resetTimer();
                    }

                    if (transfer.currentState == Transfer.State.IDLE &&
                            pathTimer.getElapsedTimeSeconds() > 2.8) {
                        intake.stopped();
                        transfer.startKickSequenceRandomly();
                    }

                    if (transfer.currentState == Transfer.State.DONE) {
                        if (!reset) {
                            resetEverything();
                        }

                        intake.forward();

                        follower.followPath(goToHumanPlayerZone, 0.65, true);
                        pathTimer.resetTimer();
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy() &&
                    pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    reset = false;
                    follower.followPath(goToScore2, true);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    intake.reverse();
                    flywheel.setVelForFarTip();

                    if(!timerReset) {
                        timerReset = true;
                        pathTimer.resetTimer();
                    }

                    if (transfer.currentState == Transfer.State.IDLE &&
                        pathTimer.getElapsedTimeSeconds() > 2.8) {
                        intake.stopped();
                        transfer.startKickSequenceRandomly();
                    }

                    if (transfer.currentState == Transfer.State.DONE) {
                        resetEverything();
                        setPathState(-1);
                    }
                }
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

        transfer.update();
        flywheel.update();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addLine();

        telemetry.addData("Limelight id: ", transfer.id);
        telemetry.update();
    }

    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        limelight = new Limelight(hardwareMap);

        startPose = startPose.mirror();
        scanAndShootPreload = scanAndShootPreload.mirror();
        pickup1EndPose = pickup1EndPose.mirror();
        pickup1ControlPose = pickup1ControlPose.mirror();
        humanPlayerPose = humanPlayerPose.mirror();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    private void resetEverything() {
        timerReset = false;
        transfer.reset();
        limelight.stop();
        flywheel.stopFlywheel();
        reset = true;
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
