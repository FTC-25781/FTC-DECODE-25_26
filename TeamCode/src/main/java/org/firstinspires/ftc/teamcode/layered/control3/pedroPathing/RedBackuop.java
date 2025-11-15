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

@Autonomous(name = "Red Far", group = "Blue")
@Configurable
public class RedBackuop extends OpMode {
    private ServoForSorter servo;
    private ServoForTransfer servo_t;
    private ShooterV2 shooter;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(84, 8, Math.toRadians(90));
    private final Pose scanPose = new Pose(115, 38, Math.toRadians(90));
    private final Pose shootPose = new Pose(80, 80, Math.toRadians(45));
    private PathChain getScan, shootPreload;
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
                setPathState(-1);
                break;

            case 1:
                // Vision-based centering on obelisk
                if (!follower.isBusy()) {


                    if ((pathTimer.getElapsedTime() >= 1.0) ||
                            pathTimer.getElapsedTime() >= 4.0) {
                        follower.followPath(shootPreload, true);
                        pathTimer.resetTimer();
                        shooter.shoot(0.7);
                        setPathState(2);
                    }
                }
                break;

            case 2:
//                if (!follower.isBusy()) {


                servo.goTo2();
                shooter.shoot(shooter.calculateTargetPower(shooter.targetRPM(follower)));


                if (pathTimer.getElapsedTime() <= 3) {
                        servo_t.moveUp();
                    }
                    else if (pathTimer.getElapsedTime() < 4) {
                        servo_t.moveDown();
                    }

                    else if (pathTimer.getElapsedTime() <= 5) {
                        servo.goTo0();
                        shooter.shoot(shooter.calculateTargetPower(shooter.targetRPM(follower)));
                    }
                    else if (pathTimer.getElapsedTime() <= 6) {
                        servo_t.moveUp();
                    }
                    else if (pathTimer.getElapsedTime() < 7) {
                        servo_t.moveDown();
                    }

                    else if (pathTimer.getElapsedTime() <= 8) {
                        servo.goTo1();
                        shooter.shoot(shooter.calculateTargetPower(shooter.targetRPM(follower)));
                    }
                    else if (pathTimer.getElapsedTime() <= 9) {
                        servo_t.moveUp();
                    }
                    else if (pathTimer.getElapsedTime() < 10) {
                        servo_t.moveDown();
                    }

                    setPathState(2);
//                    else if (pathTimer.getElapsedTime() > 11) {
//                        setPathState(3);
//                    }
//                }
                break;

            case 3:
                servo_t.moveUp();
//                if (servo_t)
                servo_t.moveDown();
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