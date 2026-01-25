package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.layeredFinal.control.Intake;
import org.firstinspires.ftc.teamcode.layeredFinal.control.Transfer;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Flywheel;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Limelight;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Turret;

@Autonomous(name = "Blue Auto Top", group = "Red")
public class BlueAutoTop extends OpMode {
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
    private boolean isRed = false;

    private final Pose startPose = new Pose(21.000, 122.000, Math.toRadians(135));
    private final Pose scanPose = new Pose(48.000, 94.000, Math.toRadians(135));
    private final Pose pick1Pose = new Pose(18.000, 84.000, Math.toRadians(180));
    private final Pose pick1ControlPose = new Pose(72.000, 83.000, Math.toRadians(0));
    private final Pose pick2Pose = new Pose(16.000, 60.000, Math.toRadians(180));
    private final Pose pick2ControlPose = new Pose(86.000, 57.000, Math.toRadians(0));
    private final Pose shoot2Pose = new Pose(62.000, 82.000, Math.toRadians(180));
    private final Pose shoot2ControlPose = new Pose(59.000, 58.000, Math.toRadians(0));

    public PathChain ScanAndShootPreload;
    public PathChain GoToPickup1;
    public PathChain ShootPick1;
    public PathChain GoToPick2;
    public PathChain ShootPick2;

    public void buildPaths() {
        ScanAndShootPreload = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                scanPose
                        )
                ).setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        GoToPickup1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                scanPose,
                                pick1ControlPose,
                                pick1Pose
                        )
                ).setLinearHeadingInterpolation(scanPose.getHeading(), pick1Pose.getHeading())
                .build();

        ShootPick1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                pick1Pose,
                                pick1ControlPose,
                                scanPose
                        )
                ).setConstantHeadingInterpolation(pick1Pose.getHeading())
                .build();

        GoToPick2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                scanPose,
                                pick2ControlPose,
                                pick2Pose
                        )
                ).setConstantHeadingInterpolation(pick2Pose.getHeading())
                .build();

        ShootPick2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                pick2Pose,
                                shoot2ControlPose,
                                shoot2Pose
                        )
                ).setConstantHeadingInterpolation(shoot2Pose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(ScanAndShootPreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    flywheel.setVelForCloseTip();

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

                        follower.followPath(GoToPickup1, true);
                        pathTimer.resetTimer();
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (!follower.isBusy() &&
                        pathTimer.getElapsedTimeSeconds() >= 2) {
                    reset = false;

                    follower.followPath(ShootPick1, true);
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

                        follower.followPath(GoToPick2, true);
                        pathTimer.resetTimer();
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy() &&
                        pathTimer.getElapsedTimeSeconds() >= 2) {
                    reset = false;

                    follower.followPath(ShootPick2, true);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    intake.reverse();

                    if (!timerReset) {
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

                        stop();
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

        follower.setStartingPose(startPose);

        Turret.lastAutoPosition = 0;

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
        Turret.lastAutoPosition = turret.turretOrientation.encoder.getCurrentPosition();
    }
}
