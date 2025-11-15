package org.firstinspires.ftc.teamcode.layered.control3.pedroPathing;

import android.content.ContentValues;
import android.database.sqlite.SQLiteDatabase;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.layered.PositionContract;
import org.firstinspires.ftc.teamcode.layered.PositiondbHelper;
import org.firstinspires.ftc.teamcode.layered.logical2.ShooterV2;
import org.firstinspires.ftc.teamcode.layered.physical1.ServoForSorter;
import org.firstinspires.ftc.teamcode.layered.physical1.ServoForTransfer;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Red Auto Far Dont use", group = "Blue")
@Configurable
public class RedAutoFar extends OpMode {
    private ServoForSorter servo;
    private ServoForTransfer servo_t;
    private ShooterV2 shooter;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(84, 8, Math.toRadians(90));
    private final Pose scanPose = new Pose(72+12, 20, Math.toRadians(67.406));
    private final Pose shootPose = new Pose(72+35, 33, Math.toRadians(90));
    private PathChain getScan, shootPreload;
    private boolean lastDpadRight = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadLeft = false;
    private boolean lastLTrigger = false;
    private boolean lastRTrigger = false;
    private Limelight3A limelight;

    ArrayList<Integer> tags = new ArrayList<Integer>();
    static int obeliskValue = 0;

    // Vision centering variables
    private boolean isCentered = false;
    private static final double CENTER_TOLERANCE = 3.0;
    private static final double STRAFE_CORRECTION_GAIN = 0.09; // adjust this value

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
                servo_t.moveDown();
                setPathState(1);
                break;

            case 1:
                // Vision-based centering on obelisk
                if (!follower.isBusy()) {


                    if ((pathTimer.getElapsedTime() >= 1.0) ||
                            pathTimer.getElapsedTime() >= 4.0) {
//                        follower.followPath(shootPreload, true);
                        pathTimer.resetTimer();
                        shooter.shoot(0.89);
                        servo.goTo2();
                        servo.update(telemetry);
                        pathTimer.resetTimer();

                        setPathState(2);

                    }
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds()>4) {

                    servo_t.moveUp();
                    pathTimer.resetTimer();


                    setPathState(4);
                }

                break;
//
//            case 3:
//                follower.followPath(shootPreload, true);
//                pathTimer.resetTimer();
//
//                setPathState(4);
//                break;

            case 4:
                if (pathTimer.getElapsedTimeSeconds()>2) {
                    servo_t.moveDown();
                    pathTimer.resetTimer();

                    setPathState(5);
                }
                break;

            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    servo.goTo0();
                    servo.update(telemetry);
                    pathTimer.resetTimer();

                    setPathState(6);
                }

                break;

            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    servo_t.moveUp();
                    pathTimer.resetTimer();

                    setPathState(7);
                }

                break;

            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    servo_t.moveDown();
                    pathTimer.resetTimer();

                    setPathState(8);
                }

                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    servo.goTo1();
                    servo.update(telemetry);
                    pathTimer.resetTimer();

                    setPathState(9);
                }

                 break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    servo_t.moveUp();
                    pathTimer.resetTimer();

                    setPathState(10);
                }
                break;

            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(shootPreload, true);
                    servo_t.moveDown();
                    pathTimer.resetTimer();

                    setPathState(11);
                }

                break;

            case 11:
                if (!follower.isBusy()) {
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

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                double strafeDistance = fiducial.getRobotPoseTargetSpace().getPosition().y;
                double horizontalOffset = fiducial.getTargetXDegrees();

                telemetry.addData("Fiducial " + id, "Distance: %.2f m, Offset: %.2f°",
                        strafeDistance, horizontalOffset);

                if (obeliskValue == 0 && tags.contains(id)) {
                    obeliskValue = id;
                }
            }
        }

        servo_t.update();
        servo.update(telemetry);
        shooter.update(telemetry);

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Obelisk Value", obeliskValue);
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
//        telemetry.addData("Is Centered", isCentered);
//        telemetry.addData("Path Timer", "%.2f sec", pathTimer.getElapsedTime());
//        telemetry.addData("Follower Busy", follower.isBusy());

        telemetry.update();
    }

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        tags.add(21);
        tags.add(22);
        tags.add(23);

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        shooter = new ShooterV2(hardwareMap);
        servo = new ServoForSorter(hardwareMap);
        servo_t = new ServoForTransfer(hardwareMap);


    }

    @Override
    public void init_loop() {
        servo_t.moveDown();

        if (gamepad2.dpad_right && !lastDpadRight) {
            servo.GoForwards();
        }

        if (gamepad2.dpad_left && !lastDpadLeft ) {
            servo.GoBackwards();
        }

        lastDpadRight = gamepad2.dpad_right;
        lastDpadLeft = gamepad2.dpad_left;
        lastDpadUp = gamepad2.dpad_up;

        lastRTrigger = gamepad2.right_bumper;
        lastLTrigger = gamepad2.left_bumper;
        servo.update(telemetry);

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        actionTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        Pose finalPose = follower.getPose();

        PositiondbHelper positiondbHelper = new PositiondbHelper(hardwareMap.appContext);
        SQLiteDatabase db = positiondbHelper.getWritableDatabase();

        db.delete(PositionContract.PositionEntry.TABLE_NAME, null, null);

        ContentValues values = new ContentValues();
        values.put(PositionContract.PositionEntry.COLUMN_NAME_X, finalPose.getX());
        values.put(PositionContract.PositionEntry.COLUMN_NAME_Y, finalPose.getY());
        values.put(PositionContract.PositionEntry.COLUMN_NAME_HEADING, finalPose.getHeading());

        long newRowId = db.insert(PositionContract.PositionEntry.TABLE_NAME, null, values);

        db.close();
        positiondbHelper.close();
    }
}