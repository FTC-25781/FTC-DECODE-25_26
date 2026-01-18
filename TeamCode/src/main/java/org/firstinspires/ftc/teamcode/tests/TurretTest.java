package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.layeredFinal.logical.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="Turret Tester Refined")
public class TurretTest extends OpMode {

    Follower follower;
    public Turret turret;

    // Toggle logic variables
    boolean isRed = true;
    boolean lastInputA = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(135)));

        // Initialize turret
        turret = new Turret(follower, hardwareMap);
        turret.setAlliance(isRed);
        turret.startAutoAlign();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(false);
    }

    @Override
    public void loop() {
        follower.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );

        if (gamepad1.a && !lastInputA) {
            isRed = !isRed;
            turret.setAlliance(isRed);
        }
        lastInputA = gamepad1.a;

        if (gamepad1.b) {
            turret.turretOrientation.encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turret.turretOrientation.encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        turret.update(); // pass current robot heading

        telemetry.addLine("Press A to Switch Alliance");
        telemetry.addLine("Press B to Reset Encoders");
        telemetry.addLine("");

        telemetry.addData("TARGET", isRed ? "RED" : "BLUE");
        telemetry.addData("On Target?", turret.isOnTarget() ? "YES" : "NO");

        double currentAngle = Math.toDegrees(turret.turretOrientation.getTurretAngle());
        telemetry.addData("Turret Angle", "%.1f deg", currentAngle);
        telemetry.addData("Encoder Ticks", turret.turretOrientation.encoder.getCurrentPosition());
        telemetry.addData("Motor Power", "%.2f", turret.turretOrientation.encoder.getPower());
        telemetry.addData("Error, ", turret.turretOrientation.calculateError());

        telemetry.addData("Robot X", "%.1f", follower.getPose().getX());
        telemetry.addData("Robot Y", "%.1f", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();
    }
}
