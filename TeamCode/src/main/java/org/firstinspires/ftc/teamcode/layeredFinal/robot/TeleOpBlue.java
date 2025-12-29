package org.firstinspires.ftc.teamcode.layeredFinal.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class TeleOpBlue extends OpMode {
    private Follower follower;
    public static Pose startingPose;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private DcMotorEx intake;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = hardwareMap.get(DcMotorEx.class, "imot");
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

       follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
       );

       if (gamepad1.a) {
           intake.setPower(1);
       }

       if (gamepad1.b) {
           intake.setPower(0);
       }

       if (gamepad1.y) {
           intake.setPower(-1);
       }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());

    }
}