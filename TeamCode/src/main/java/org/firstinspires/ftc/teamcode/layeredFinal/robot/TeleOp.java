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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// TODO: Check why TeleOp is so delayed
@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Blue", group = "TeleOp")
public class TeleOp extends OpMode {
    private Intake intake;
    private Transfer transfer;
    private Flywheel deposit;
    private Limelight limelight;

    private Follower follower;
    public static Pose startingPose;
    private TelemetryManager telemetryM;

    private boolean lastLeftTrigger = false;
    private boolean lastRightTrigger = false;

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

        transfer.id = limelight.getLastLoggedID();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        transfer.update();
        follower.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric Mode
        );

        if (gamepad1.aWasPressed()) { intake.forward(); }
        if (gamepad1.bWasPressed()) { intake.stopped(); }
        if (gamepad1.xWasPressed()) { intake.reverse(); }


        if (gamepad1.dpadLeftWasPressed()) {
            if (!transfer.isFiringRandomly() && !transfer.isFiringInOrder()) {
                transfer.startKickSequenceInOrder();
            }
        }

        if (gamepad1.dpadRightWasPressed()) {
            if (!transfer.isFiringRandomly() && !transfer.isFiringInOrder()) {
                transfer.startKickSequenceRandomly();
            }
        }

        if (gamepad1.yWasPressed()) {
            transfer.reset();
        }

        if (gamepad1.dpadUpWasPressed()) { deposit.setVelForCloseTip(); }
        if (gamepad1.dpadDownWasPressed()) { deposit.setVelForFarTip(); }
        if (gamepad1.leftStickButtonWasPressed()) { deposit.stopFlywheel(); }
        if (gamepad1.rightStickButtonWasPressed()) { deposit.humanPlayer(); }

        if (gamepad1.leftBumperWasPressed()) { deposit.updateHighVelocity(-10); }
        if (gamepad1.rightBumperWasPressed()) { deposit.updateHighVelocity(10); }

        boolean currentLeftTrigger = gamepad1.left_trigger > 0.5;
        if (currentLeftTrigger && !lastLeftTrigger) {
            deposit.updateHighVelocity(-10);
        }
        lastLeftTrigger = currentLeftTrigger;

        boolean currentRightTrigger = gamepad1.right_trigger > 0.5;
        if (currentRightTrigger && !lastRightTrigger) {
            deposit.updateHighVelocity(10);
        }
        lastRightTrigger = currentRightTrigger;

        deposit.update();

        telemetryM.debug("Position", follower.getPose());
        telemetryM.debug("Velocity", follower.getVelocity());
        telemetryM.update();
    }
}
