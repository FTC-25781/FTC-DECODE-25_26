package org.firstinspires.ftc.teamcode.layeredFinal.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.layeredFinal.control.Intake;
import org.firstinspires.ftc.teamcode.layeredFinal.control.Transfer;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Flywheel;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "TeleOp Blue", group = "TeleOp")
public class TeleOpBlue extends OpMode {
    // Subsystem instances
    private Intake intake;
    private Transfer transfer;
    private Flywheel deposit;

    // Navigation and Telemetry
    private Follower follower;
    public static Pose startingPose; // Static so Auto can pass the final position to TeleOp
    private TelemetryManager telemetryM;

    // State tracking for Trigger Edge Detection
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
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Essential periodic updates
        transfer.updateColors(); // Check sensors for element detection
        follower.update();      // Update localizer and pathing
        telemetryM.update();    // Send data to dashboard/driver station

        // Drivetrain control: -y is forward, -x is left, -rx is clockwise
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric Mode
        );

        // --- Intake Commands ---
        if (gamepad1.aWasPressed()) { intake.forward(); }
        if (gamepad1.bWasPressed()) { intake.stopped(); }
        if (gamepad1.xWasPressed()) { intake.reverse(); }

        // --- Transfer Commands (Sequencing elements) ---
        if (gamepad1.dpadLeftWasPressed()) { transfer.shootSequential(); }
        if (gamepad1.dpadRightWasPressed()) { transfer.shootInOrder(); }

        // --- Flywheel Preset Commands ---
        if (gamepad1.dpadUpWasPressed()) { deposit.setVelForCloseTip(); }
        if (gamepad1.dpadDownWasPressed()) { deposit.setVelForFarTip(); }
        if (gamepad1.leftStickButtonWasPressed()) { deposit.stopFlywheel(); }
        if (gamepad1.rightStickButtonWasPressed()) { deposit.humanPlayer(); }

        // --- Velocity Fine-Tuning (Bumpers) ---
        if (gamepad1.leftBumperWasPressed()) { deposit.updateHighVelocity(-10); }
        if (gamepad1.rightBumperWasPressed()) { deposit.updateHighVelocity(10); }

        // --- Trigger Edge Detection (Rising Edge) ---
        // This prevents the velocity from changing every single frame while the trigger is held.

        // Handle Left Trigger (Decrease Velocity)
        boolean currentLeftTrigger = gamepad1.left_trigger > 0.5; // Threshold used to treat analog as a button
        if (currentLeftTrigger && !lastLeftTrigger) {
            deposit.updateHighVelocity(-10); // Runs only once when trigger passes 0.5
        }
        lastLeftTrigger = currentLeftTrigger; // Update state for next loop

        // Handle Right Trigger (Increase Velocity)
        boolean currentRightTrigger = gamepad1.right_trigger > 0.5;
        if (currentRightTrigger && !lastRightTrigger) {
            deposit.updateHighVelocity(10); // Runs only once when trigger passes 0.5
        }
        lastRightTrigger = currentRightTrigger;

        // Periodic update for Flywheel PIDF/Control loop
        deposit.update();

        // Debugging info
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
    }
}
