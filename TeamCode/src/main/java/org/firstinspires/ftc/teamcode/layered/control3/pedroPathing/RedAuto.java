package org.firstinspires.ftc.teamcode.layered.control3.pedroPathing; // make sure this aligns with class location

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
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import android.content.ContentValues;
import android.database.sqlite.SQLiteDatabase;

//import org.firstinspires.ftc.teamcode.layered.logical2.Shooter;
import org.firstinspires.ftc.teamcode.layered.PositionContract;
import org.firstinspires.ftc.teamcode.layered.PositiondbHelper;
import org.firstinspires.ftc.teamcode.layered.physical1.ServoForSorter;
import org.firstinspires.ftc.teamcode.layered.physical1.ServoForTransfer;
import org.firstinspires.ftc.teamcode.layered.logical2.ShooterV2;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Red Auto", group = "Blue")
@Configurable
public class RedAuto extends OpMode {
    //    private Shooter shooter;
    private ServoForSorter servo;
    private ServoForTransfer servo_t;
    private ShooterV2 shooter;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(122, 122, Math.toRadians(45));
    private final Pose scanPose = new Pose(72, 72, Math.toRadians(90));
    private final Pose shootPose = new Pose(80, 80, Math.toRadians(45));
    private PathChain getScan, shootPreload;
    private Limelight3A limelight;

    ArrayList<Integer> tags = new ArrayList<Integer>();

    static int obeliskValue = 0;


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
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTime() >= 3) {



//                    if (actionTimer.getElapsedTime() == 1.0) {
//
//                    }
//
//                    if (actionTimer.getElapsedTime() == 0.2) {
//                        servo_t.moveUp();
//                    }

                    follower.followPath(shootPreload, true);
                    setPathState(2);
                }
                break;
            case 2: // GPP, id 21

                if(pathTimer.getElapsedTime() < 3){
                    servo.goTo2();
                    shooter.shoot(shooter.calculateTargetPower(shooter.targetRPM(follower)));
                }
                if(pathTimer.getElapsedTime() >= 3 && pathTimer.getElapsedTime() <= 3.1){
                    servo_t.moveUp();
                }
                if(pathTimer.getElapsedTime() > 3.1 && pathTimer.getElapsedTime() < 3.2){
                    servo_t.moveDown();
                }
                if(pathTimer.getElapsedTime() >= 3.5 && pathTimer.getElapsedTime() <= 3.6){
                    servo.goTo0();
                    shooter.shoot(shooter.calculateTargetPower(shooter.targetRPM(follower)));
                }
                if(pathTimer.getElapsedTime() >= 5.5 && pathTimer.getElapsedTime() <= 5.6){
                    servo_t.moveUp();
                }
                if(pathTimer.getElapsedTime() > 5.8 && pathTimer.getElapsedTime() < 6){
                    servo_t.moveDown();
                }
                if(pathTimer.getElapsedTime() >= 6.2 && pathTimer.getElapsedTime() <= 6.4){
                    servo.goTo1();
                    shooter.shoot(shooter.calculateTargetPower(shooter.targetRPM(follower)));
                }
                if(pathTimer.getElapsedTime() >= 8.4 && pathTimer.getElapsedTime() <= 8.5){
                    servo_t.moveUp();
                }
                if(pathTimer.getElapsedTime() > 8.8 && pathTimer.getElapsedTime() < 8.9){
                    servo_t.moveDown();
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

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            double StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getPosition().y;
            telemetry.addData("Fiducial " + id, "is " + StrafeDistance_3D + " meters away");
            if(!tags.contains(obeliskValue)){
                if(tags.contains(id)){
                    obeliskValue = id;
                }
            }
        }

//        servo_t.update();
//        servo.update(telemetry);
//        shooter.update(telemetry);

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Obelisk Value", obeliskValue);
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
//        servo = new ServoForSorter(hardwareMap);
//        servo_t = new ServoForTransfer(hardwareMap);
//        shooter = new Shooter(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        limelight.start();

        tags.add(21);
        tags.add(22);
        tags.add(23);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        shooter = new ShooterV2(hardwareMap);
        servo = new ServoForSorter(hardwareMap);
        servo_t = new ServoForTransfer(hardwareMap);
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
        servo_t.moveDown();
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        //actionTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
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

