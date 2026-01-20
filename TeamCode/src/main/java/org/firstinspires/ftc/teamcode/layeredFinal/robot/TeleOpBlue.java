package org.firstinspires.ftc.teamcode.layeredFinal.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.layeredFinal.control.Intake;
import org.firstinspires.ftc.teamcode.layeredFinal.control.Transfer;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Flywheel;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Limelight;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Blue", group = "Main")
public class TeleOpBlue extends OpMode {
    private Intake intake;
    private Transfer transfer;
    private Flywheel deposit;
    private Limelight limelight;
    private Turret turret;

    private Follower follower;
    public static Pose startingPose;
    private TelemetryManager telemetryM;
    boolean isRed = false;

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
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        transfer.update();
        follower.update();
        turret.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric Mode
        );

        if (gamepad1.aWasPressed()) { intake.forward(); }
        if (gamepad1.bWasPressed()) { intake.stopped(); }
        if (gamepad1.xWasPressed()) { intake.reverse(); }

        if (gamepad1.dpadLeftWasPressed()) { transfer.startKickSequenceInOrder(limelight.getLastLoggedID()); }
        if (gamepad1.dpadRightWasPressed()) { transfer.startKickSequenceRandomly(); }
        if (gamepad1.yWasPressed()) { transfer.reset(); }

        if (gamepad1.left_trigger > 0.1) {
            deposit.setVelForCloseTip();
            turret.startAutoAlign();
        }
        if (gamepad1.right_trigger > 0.1) {
            deposit.setVelForFarTip();
            turret.startAutoAlign();
        }

        if (gamepad1.leftStickButtonWasPressed()) {
            deposit.stopFlywheel();
            turret.stopAutoAlign();
        }

        if (gamepad1.rightStickButtonWasPressed()) { deposit.humanPlayer(); }

        deposit.update();

        telemetry.addData("Position", follower.getPose());
        telemetry.addData("Velocity", follower.getVelocity());
        telemetry.addLine("");
        telemetry.addData("Limelight Id", limelight.getLastLoggedID());
        telemetry.addData("Transfer Id", transfer.id);
        telemetry.addLine("");
        telemetry.addData("Deposit Velocity", deposit.getVelocity());
        telemetry.update();
    }
}
