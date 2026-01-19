package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.layeredFinal.logical.Turret;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.TurretTracker;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartLimelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="Turret Tester Refined")
public class TurretTest extends OpMode {

    Follower follower;
    public Turret turret;
    public SmartLimelight limelight;
    // Toggle logic variables
    boolean isRed = true;
    boolean lastInputA = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));

        // Initialize turret
        turret = new Turret(follower, hardwareMap);
        turret.setAlliance(isRed);
        turret.startAutoAlign();

        limelight = new SmartLimelight(hardwareMap);

    }

    @Override
    public void start() {
        follower.startTeleopDrive(false);
    }

    @Override
    public void loop() {
        follower.update();
        turret.update();

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


        telemetry.addData("TARGET", isRed ? "RED" : "BLUE");
        telemetry.addData("Turret Angle", "%.1f deg", turret.turretOrientation.turretGolbalAngle());
        telemetry.addData("Encoder Ticks", turret.turretOrientation.encoder.getCurrentPosition());
        telemetry.addData("Robot Heading", "%.1f deg", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Angle to Goal", turret.turretOrientation.getAngleToGoal());
        telemetry.addData("Desired turret angle", turret.turretOrientation.calculateDesiredTurretAngle());
        telemetry.addData("PID Error", turret.turretPID.getError());

        telemetry.update();
    }
}