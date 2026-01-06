package org.firstinspires.ftc.teamcode.layeredFinal.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.layeredFinal.control.Intake;
import org.firstinspires.ftc.teamcode.layeredFinal.control.Transfer;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Flywheel;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "TeleOp Blue", group = "TeleOp")
public class TeleOpBlue extends OpMode {
    // Subsystem instances
    private Intake intake;
    private Transfer transfer;
    private Flywheel deposit;
    private Limelight limelight;

    // Navigation and Telemetry
    private Follower follower;
    public static Pose startingPose; // Static so Auto can pass the final position to TeleOp
    private TelemetryManager telemetryM;

    // Debounce booleans
    private boolean lastLeftTrigger = false;
    private boolean lastRightTrigger = false;

    @Override
    public void init() {
        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        // Custom Telemetry setup
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize hardware abstractions
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
        telemetryM.update();

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
            if (!transfer.isFiring()) {
                transfer.startKickSequenceInOrder();
            }
        }

        if (gamepad1.dpadRightWasPressed()) {
            if (!transfer.isFiring()) {
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

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.update();
    }
}
