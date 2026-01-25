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

@Config
@Configurable
@TeleOp(name = "TeleOp Red", group = "Main")
public class TeleOpRed extends OpMode {
    private Intake intake;
    private Transfer transfer;
    private Flywheel deposit;
    private Limelight limelight;
    private Turret turret;

    private Follower follower;
    private static Pose startingPose = new Pose(72, 72, Math.toRadians(90));
    private TelemetryManager telemetryM;
    private boolean isRed = true;
    Timer loopTimer = new Timer();


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
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
        loopTimer.resetTimer();
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

        if (gamepad1.left_trigger > 0.1) {
            deposit.setVelForCloseTip();
            if (!turret.autoAlign) {
                turret.startAutoAlign();
            }
        }
        if (gamepad1.right_trigger > 0.1) {
            deposit.setVelForFarTip();
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

        deposit.update();
        turret.update();
        transfer.update();

        telemetry.addData("Position", follower.getPose());
        telemetry.addData("Velocity", follower.getVelocity());
        telemetry.addData("Limelight Id", limelight.getLastLoggedID());
        telemetry.addData("Transfer Id", transfer.id);
        telemetry.addData("Kicker With Green", transfer.kickerWithGreen);
        telemetry.addData("Deposit Velocity", deposit.getVelocity());
        telemetry.addData("Turret Error", turret.turretPID.getError());
        telemetry.addData("Turret Angle to Goal", turret.turretOrientation.getAngleToGoal());
        telemetry.addData("Turret Angle from Encoder", turret.turretOrientation.encoder.getCurrentPosition());
        telemetry.addData("Loop time", loopTimer.getElapsedTime());
        telemetry.update();
    }
}
