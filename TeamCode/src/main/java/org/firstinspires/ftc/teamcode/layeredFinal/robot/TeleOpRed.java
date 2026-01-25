package org.firstinspires.ftc.teamcode.layeredFinal.robot;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.layeredFinal.control.Intake;
import org.firstinspires.ftc.teamcode.layeredFinal.control.Transfer;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Flywheel;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Limelight;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Configurable
@TeleOp(name = "TeleOp Red", group = "Main")
public class TeleOpRed extends OpMode {
    private Intake intake;

    public static int lastAutoPosition = 0;

    private Transfer transfer;
    private Flywheel deposit;
    private Limelight limelight;
    private Turret turret;
    private Follower follower;
    private static Pose startingPose;
    private TelemetryManager telemetryM;
    private boolean isRed = true;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        deposit = new Flywheel(hardwareMap);
        limelight = new Limelight(hardwareMap);
        turret = new Turret(follower, hardwareMap);
        turret.setAlliance(isRed);
        turret.startAutoAlign();

        limelight.stop();

    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric Mode
        );

        if (gamepad1.aWasPressed()) {
            intake.forward();
        }

        if (gamepad1.bWasPressed()) {
            intake.stopped();
        }

        if (gamepad1.xWasPressed()) {
            intake.reverse();
        }

        if (gamepad1.dpadLeftWasPressed()) {
            transfer.updateColorSensor();
            transfer.startKickSequenceInOrder(limelight.getLastLoggedID());
        }

        if (gamepad1.dpadRightWasPressed()) {
            transfer.startKickSequenceRandomly();
        }

        if (gamepad1.yWasPressed()) {
            transfer.reset();
        }
        if(gamepad1.dpadDownWasPressed()){
            turret.stopAutoAlign();
            turret.turretOrientation.encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turret.turretOrientation.encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            turret.turretPID.setTargetPosition(0);
        }
        if (gamepad1.left_trigger > 0.1) {
            deposit.setVelForCloseTip();
            if(turret.autoAlign){
                turret.stopAutoAlign();
            }
            if (!turret.autoAlign) {
                turret.startAutoAlign();
            }
        }
        if (gamepad1.right_trigger > 0.1) {
            deposit.setVelForFarTip();
            if(turret.autoAlign){
                turret.stopAutoAlign();
            }
            if (!turret.autoAlign) {
                turret.startAutoAlign();
            }
        }

        if (gamepad1.leftStickButtonWasPressed()) {
            deposit.stopFlywheel();
            turret.stopAutoAlign();
        }

        if (gamepad1.rightStickButtonWasPressed()) {
            deposit.humanPlayer();
        }
        if(gamepad1.dpadUpWasPressed()){
            turret.turretOrientation.encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turret.turretOrientation.encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            turret.turretPID.setTargetPosition(0);
            //turret.turretOrientation.setLocalAngle(0);
        }
        deposit.update();
        transfer.update();
        turret.update();

        telemetry.addData("Position", follower.getPose());
        telemetry.addData("Position", follower.getPose());
        // telemetry.addData("Velocity", follower.getVelocity());
        //telemetry.addData("Limelight Id", limelight.getLastLoggedID());
        //telemetry.addData("Transfer Id", transfer.id);
        //telemetry.addData("Kicker With Green", transfer.kickerWithGreen);
        //telemetry.addData("Deposit Velocity", deposit.getVelocity());
        telemetry.addData("Turret Error", turret.turretPID.getError());
        telemetry.addData("Turret Angle to Goal", turret.turretOrientation.getAngleToGoal());
        telemetry.addData("Turret Angle from Encoder", turret.turretOrientation.encoder.getCurrentPosition());
        //telemetry.addData("Loop time", loopTimer.getElapsedTime());
        telemetry.update();
    }
}
