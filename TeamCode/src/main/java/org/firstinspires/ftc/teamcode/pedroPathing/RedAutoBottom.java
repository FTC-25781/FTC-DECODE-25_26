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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.layeredFinal.control.Intake;
import org.firstinspires.ftc.teamcode.layeredFinal.control.Transfer;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Flywheel;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Limelight;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Turret;

@Autonomous(name = "Red Auto Bottom", group = "Red")
public class RedAutoBottom extends OpMode {
    private Intake intake;
    private Transfer transfer;
    private Limelight limelight;
    private Flywheel flywheel;
    private Turret turret;

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState;

    private boolean timerReset = false;
    private boolean reset = false;
    private boolean isRed = true;

    private final Pose startPose = new Pose(95.8, 8, Math.toRadians(90));
    private final Pose scanAndShootPreload = new Pose(94, 9, Math.toRadians(90));
    private final Pose homePose = new Pose(92, 12, Math.toRadians(0));
    private final Pose pickup1EndPose = new Pose(131,40.5, Math.toRadians(0));
    private final Pose pickup1ControlPose = new Pose(64, 42, Math.toRadians(0));
    private final Pose humanPlayerPose = new Pose(127, 8, Math.toRadians(0));
    private final Pose openTheGate = new Pose(129, 70, Math.toRadians(90));
    private final Pose openTheGateControlPose = new Pose(80, 75, Math.toRadians(0));
    private final Pose getBallsFromGate = new Pose(131, 56, Math.toRadians(50));
    private final Pose getBallsFromGateControlPose = new Pose(128, 35, Math.toRadians(0));

    private Path scanAndShoot;
    private PathChain goToPickup1, goToScore, goToHumanPlayerZone, goToScore2, goToOpenTheGate, goToGetBalls, goToScoreGate;

    public void buildPaths() {
        scanAndShoot = new Path(new BezierLine(startPose, scanAndShootPreload));
        scanAndShoot.setConstantHeadingInterpolation(scanAndShootPreload.getHeading());

        goToPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scanAndShootPreload, pickup1ControlPose, pickup1EndPose))
                .setLinearHeadingInterpolation(scanAndShootPreload.getHeading(), pickup1EndPose.getHeading())
                .build();

        goToScore = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1EndPose, pickup1ControlPose, homePose))
                .setConstantHeadingInterpolation(homePose.getHeading())
                .build();

        goToHumanPlayerZone = follower.pathBuilder()
                .addPath(new BezierLine(homePose, humanPlayerPose))
                .setConstantHeadingInterpolation(humanPlayerPose.getHeading())
                .build();

        goToScore2 = follower.pathBuilder()
                .addPath(new BezierLine(humanPlayerPose, homePose))
                .setConstantHeadingInterpolation(homePose.getHeading())
                .build();

        goToOpenTheGate = follower.pathBuilder()
                .addPath(new BezierCurve(homePose, openTheGateControlPose, openTheGate))
                .setLinearHeadingInterpolation(homePose.getHeading(), openTheGate.getHeading())
                .build();

        goToGetBalls = follower.pathBuilder()
                .addPath(new BezierCurve(openTheGate, getBallsFromGateControlPose, getBallsFromGate))
                .setLinearHeadingInterpolation(openTheGate.getHeading(), getBallsFromGate.getHeading())
                .build();

        goToScoreGate = follower.pathBuilder()
                .addPath(new BezierCurve(getBallsFromGate, openTheGateControlPose, homePose))
                .setConstantHeadingInterpolation(getBallsFromGate.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                limelight.stop();
                follower.followPath(scanAndShoot);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1) {
                    flywheel.setVelForFarTip();

                    if(!timerReset) {
                        timerReset = true;
                        pathTimer.resetTimer();
                    }

                    if (transfer.currentState == Transfer.State.IDLE &&
                        pathTimer.getElapsedTimeSeconds() > 2.2) {
                        transfer.startKickSequenceRandomly();
                    }

                    if (transfer.currentState == Transfer.State.DONE) {
                        if (!reset) {
                            resetEverything();
                            intake.forward();
                        }

                        follower.followPath(goToPickup1,0.8, true);
                        pathTimer.resetTimer();
                        setPathState(2);
                    }
                }

                break;
            case 2:
                if (!follower.isBusy() &&
                    pathTimer.getElapsedTimeSeconds() >= 2) {
                    reset = false;

                    follower.followPath(goToScore, true);
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    intake.reverse();

                    if(!timerReset) {
                        timerReset = true;
                        pathTimer.resetTimer();
                    }

                    if (transfer.currentState == Transfer.State.IDLE
                        && pathTimer.getElapsedTimeSeconds() >= 0.5) {
                        transfer.startKickSequenceRandomly();
                    }

                    if (transfer.currentState == Transfer.State.DONE) {
                        if (!reset) {
                            resetEverything();
                            intake.forward();
                        }

                        follower.followPath(goToHumanPlayerZone, 0.8, true);
                        pathTimer.resetTimer();
                        setPathState(4);
                      }
                }
                break;

            case 4:
                if (!follower.isBusy() &&
                    pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    reset = false;

                    follower.followPath(goToScore2, 0.8, true);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;

            case 5:
                if(!follower.isBusy() &&
                    pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    intake.reverse();

                    if(!timerReset) {
                        timerReset = true;
                        pathTimer.resetTimer();
                    }

                    if (transfer.currentState == Transfer.State.IDLE &&
                        pathTimer.getElapsedTimeSeconds() >= 0.5) {
                        transfer.startKickSequenceRandomly();
                    }

                    if (transfer.currentState == Transfer.State.DONE) {
                        if (!reset) {
                            resetEverything();
                            intake.forward();
                        }

                        follower.followPath(goToOpenTheGate,0.65, true);
                        pathTimer.resetTimer();
                        setPathState(-1);
                    }
                }
                break;

            case 6:
                if (!follower.isBusy() &&
                    pathTimer.getElapsedTimeSeconds() >= 2.5) {
                    reset = false;

                    follower.followPath(goToGetBalls, true);
                    pathTimer.resetTimer();
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy() &&
                    pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    follower.followPath(goToScoreGate, true);
                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    intake.reverse();

                    if(!timerReset) {
                        timerReset = true;
                        pathTimer.resetTimer();
                    }

                    if (transfer.currentState == Transfer.State.IDLE &&
                        pathTimer.getElapsedTimeSeconds() >= 0.5) {
                        transfer.startKickSequenceRandomly();
                    }

                    if (transfer.currentState == Transfer.State.DONE) {
                        if (!reset) {
                            resetEverything();
                            intake.stopped();
                            flywheel.stopFlywheel();
                        }

                        setPathState(-1);
                    }
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        follower.update();

        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        limelight = new Limelight(hardwareMap);
        turret = new Turret(follower, hardwareMap);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        turret.setAlliance(isRed);
        turret.startAutoAlign();

        limelight.getIDAndLog(limelight.getID());
        transfer.id = limelight.getLastLoggedID();

        follower.setStartingPose(startPose);

        telemetry.addData("Limelight id: ", transfer.id);
        telemetry.update();

    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        transfer.update();
        flywheel.update();
        turret.update();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addLine();

        telemetry.addData("Limelight id: ", transfer.id);
        telemetry.update();
    }

    private void resetEverything() {
        timerReset = false;
        transfer.reset();
        limelight.stop();
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
