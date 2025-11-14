package org.firstinspires.ftc.teamcode.layered.robot4;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.layered.control3.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.layered.logical2.UpdatedShooter;
import org.firstinspires.ftc.teamcode.layered.physical1.EncoderForIntake;
import org.firstinspires.ftc.teamcode.layered.physical1.ServoForSorter;
import org.firstinspires.ftc.teamcode.layered.physical1.ServoForTransfer;
import org.firstinspires.ftc.teamcode.layered.physical1.IntakeMotor;

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
    private Follower follower;
    private UpdatedShooter shooter;

    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72,45));
        follower.update();

        mot = new IntakeMotor(hardwareMap);
        servo = new ServoForSorter(hardwareMap);
        servo_t = new ServoForTransfer(hardwareMap);
        encoder = new EncoderForIntake(hardwareMap);
        shooter = new UpdatedShooter(hardwareMap);

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierPoint(follower::getPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        telemetry.addData("Status", "We go to worlds????");
        telemetry.addLine();
        telemetry.addLine("Shooter Controls:");
        telemetry.addLine("DPAD LEFT - Toggle Shooter On/Off");
        telemetry.addLine("DPAD RIGHT - Toggle Alliance (Red/Blue)");
        telemetry.addLine("DPAD DOWN (Hold) - Auto-aim at goal");
        telemetry.update();
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

        if (gamepad1.a) {
            mot.startIntaking();
        }

        if (gamepad1.b) {
            mot.returnToIdle();
        }

        if (gamepad1.x) {
            mot.startRemoving();
        }

        if (gamepad1.dpad_right && !lastDpadRight) {
            servo.GoForwards();
        }

        if (gamepad1.dpad_left && !lastDpadLeft ) {
            servo.GoBackwards();
        }

        if(gamepad1.dpad_up) {
            servo_t.moveUp();
        }

        if(gamepad1.dpad_down) {
            servo_t.moveDown();
        }

        if (gamepad1.right_trigger>0.1) {
            shooter.shoot(shooter.calculateTargetPower(shooter.targetRPM()));
            pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierPoint(follower::getPose)))
                    .setConstantHeadingInterpolation(Math.atan((132-follower.getPose().getY())/(132-follower.getPose().getX())))
                    .build();
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }
        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.left_trigger>0.1 || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        if(gamepad1.left_trigger>0.1) {
            shooter.shoot(0);
//            servo_t.moveDown();

        }

        servo_t.update();
        mot.update();
        servo.update(telemetry);
        shooter.update(telemetry);

        lastDpadRight = gamepad1.dpad_right;
        lastDpadLeft = gamepad1.dpad_left;

        telemetry.addData("Follower Pose X", follower.getPose().getX());
        telemetry.addData("Follower Pose Y", follower.getPose().getY());
        telemetry.addData("Follower Pose Head", follower.getPose().getHeading());
        telemetry.addData("Encoder Pos: ", encoder.pos());
        telemetry.update();
    }
}
