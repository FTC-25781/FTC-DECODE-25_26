package org.firstinspires.ftc.teamcode.layered.robot4;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.layered.control3.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.layered.logical2.Shooter;
import org.firstinspires.ftc.teamcode.layered.physical1.EncoderForIntake;
import org.firstinspires.ftc.teamcode.layered.physical1.ServoForSorter;
import org.firstinspires.ftc.teamcode.layered.physical1.ServoForTransfer;
import org.firstinspires.ftc.teamcode.layered.physical1.IntakeMotor;
import org.firstinspires.ftc.teamcode.layered.logical2.ShooterV2;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteException;
import org.firstinspires.ftc.teamcode.layered.PositionContract;
import org.firstinspires.ftc.teamcode.layered.PositiondbHelper;



import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Supplier;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class Robot extends OpMode {
    private IntakeMotor mot;
    private EncoderForIntake encoder;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private ServoForSorter servo;
    private ServoForTransfer servo_t;
    public double variation = 0;
    private Follower follower;
    private ShooterV2 shooter;
    private boolean lastDpadRight = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadLeft = false;
    private boolean lastLTrigger = false;
    private boolean lastRTrigger = false;

    private final Pose DEFAULT_START_POSE =      new Pose(122, 122, Math.toRadians(45));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        Pose savedPose = readSavedPose();
        follower.setStartingPose(savedPose);
        follower.update();

        mot = new IntakeMotor(hardwareMap);
        servo = new ServoForSorter(hardwareMap);
        servo_t = new ServoForTransfer(hardwareMap);
        encoder = new EncoderForIntake(hardwareMap);
        shooter = new ShooterV2(hardwareMap);

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierPoint(follower::getPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        telemetry.addData("Status", "We go to worlds????");
        telemetry.addData("staring pose", "X: %.1f, Y: %.1f, H: %.1f deg",
                savedPose.getX(), savedPose.getY(), Math.toDegrees(savedPose.getHeading()));
        telemetry.addLine();
        telemetry.addLine("Shooter Controls:");
        telemetry.addLine("DPAD LEFT - Toggle Shooter On/Off");
        telemetry.addLine("DPAD RIGHT - Toggle Alliance (Red/Blue)");
        telemetry.addLine("DPAD DOWN (Hold) - Auto-aim at goal");
        telemetry.update();
    }

    public Pose readSavedPose(){
        PositiondbHelper positiondbHelper = null;
        SQLiteDatabase db = null;
        Cursor cursor = null;
        Pose finalPose = DEFAULT_START_POSE;

        try{
            positiondbHelper = new PositiondbHelper(hardwareMap.appContext);
            db = positiondbHelper.getReadableDatabase();

            String[] projection = {
                    PositionContract.PositionEntry.COLUMN_NAME_X,
                    PositionContract.PositionEntry.COLUMN_NAME_Y,
                    PositionContract.PositionEntry.COLUMN_NAME_HEADING
            };
            cursor = db.query(
                    PositionContract.PositionEntry.TABLE_NAME,
                    projection,
                    null,
                    null,
                    null,
                    null,
                    null
            );
            if (cursor.moveToFirst()) {
                // Get column indices
                int xIndex = cursor.getColumnIndexOrThrow(PositionContract.PositionEntry.COLUMN_NAME_X);
                int yIndex = cursor.getColumnIndexOrThrow(PositionContract.PositionEntry.COLUMN_NAME_Y);
                int headingIndex = cursor.getColumnIndexOrThrow(PositionContract.PositionEntry.COLUMN_NAME_HEADING);

                // Read the real values
                double x = cursor.getDouble(xIndex);
                double y = cursor.getDouble(yIndex);
                double heading = cursor.getDouble(headingIndex);

                finalPose = new Pose(x, y, heading);
                telemetry.addLine("Loaded starting pose from Auto DB.");

            } else {
                telemetry.addLine("Auto DB empty so starting at default pose.");
            }
        } catch (SQLiteException e) {
            telemetry.addLine("Database read failed so starting at default pose" + e.getMessage());
        } finally {
            if(cursor != null) cursor.close();
            if(db != null) db.close();
            if(positiondbHelper != null) positiondbHelper.close();
        }
        return finalPose;
    }
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        // Read joystick inputs
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x*0.75,
                true
        );

        if (gamepad2.a == true && servo_t.returnDownMax()) {
            servo.current_Pos=0;
        }

        if (gamepad2.b == true &&  servo_t.returnDownMax()) {
            servo.current_Pos=1;
        }

        if (gamepad2.x == true && servo_t.returnDownMax()) {
            servo.current_Pos=2;
        }

        if (gamepad2.dpad_right && !lastDpadRight) {
            servo.GoForwards();
        }

        if (gamepad2.dpad_left && !lastDpadLeft ) {
            servo.GoBackwards();
        }

        if(gamepad2.dpad_up) {
            servo_t.moveUp();
        }
        if (!gamepad2.dpad_up && lastDpadUp){
            servo_t.moveDown();
        }

        if(gamepad2.dpad_down) {
            servo_t.moveDown();
        }

//        if (gamepad1.right_trigger>0.1) {
////            shooter.shoot(shooter.calculateTargetPower(shooter.targetRPM()));
//            shooter.shoot(shooter.calculateTargetPower(shooter.targetRPM(follower)));
//            pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
//                    .addPath(new Path(new BezierPoint(follower::getPose)))
//                    .setLinearHeadingInterpolation(follower.getPose().getPose().getHeading(), Math.atan((144-follower.getPose().getY())/(144-follower.getPose().getX())))
////                    .setLinearHeadingInterpolation(follower.getPose().getPose().getHeading(), Math.atan((132-follower.getPose().getY())/(132-follower.getPose().getX())))
//                    .build();
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }

//        if (gamepad1.right_bumper) {
//            shooter.shoot(shooter.calculateTargetPower(shooter.targetRPM(follower)));
//            pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
//                    .addPath(new Path(new BezierPoint(follower::getPose)))
//                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.atan((132-follower.getPose().getY())/(132-follower.getPose().getX())), 0.7))
//                    .build();
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }

        //Right Trigger = tip of big triangle (0.7)
        //Right Bumper = up close to backboard (0.6)
        //left trigger = all the way back (0.9)

        if (gamepad1.right_trigger >0.1) {
            shooter.shoot(0.78+variation);
        }

        if (gamepad1.right_bumper) {
            shooter.shoot(0.72+variation);
        }

        if (gamepad1.left_trigger > 0.1) {
            shooter.shoot(0.9+variation);
        }

        if (gamepad2.right_bumper && !lastRTrigger) {
            variation += 0.02;
        }

        if (gamepad2.left_bumper && !lastLTrigger) {
            variation -= 0.02;
        }

        //Stop automated following if the follower is done
//        if (automatedDrive && (gamepad1.left_trigger>0.1 || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }

        if(gamepad1.left_bumper) {
            shooter.shoot(0);
        }

        if (gamepad1.dpad_down && servo_t.returnDownMax()) {
            shooter.reverseDepositMotor();
        } else if (gamepad1.dpad_down &&servo_t.returnTopMax()){
            servo_t.moveDown();
        }

        if (gamepad2.dpad_down) {
            follower.setPose(new Pose(10, 10, 90));
        }

        servo_t.update();
        mot.update();
        servo.update(telemetry);
        shooter.update(telemetry);

        lastDpadRight = gamepad2.dpad_right;
        lastDpadLeft = gamepad2.dpad_left;
        lastDpadUp = gamepad2.dpad_up;

        lastRTrigger = gamepad2.right_bumper;
        lastLTrigger = gamepad2.left_bumper;

        telemetry.addData("Follower Pose X", follower.getPose().getX());
        telemetry.addData("Follower Pose Y", follower.getPose().getY());
        telemetry.addData("Follower Pose Head", follower.getPose().getHeading());
        telemetry.addData("Distance", Math.sqrt(Math.pow(Math.abs(follower.getPose().getX()) - 132, 2) + Math.pow(Math.abs(follower.getPose().getY()) - 132, 2)));
        telemetry.addData("Encoder Pos: ", encoder.pos());
        telemetry.addData("Idly Pos: ", servo.current_Pos);

        telemetry.addData("Variation", variation);
        telemetry.addData("Angle of robot", Math.atan((132-follower.getPose().getY())/(132-follower.getPose().getX())));
        telemetry.update();
    }
}