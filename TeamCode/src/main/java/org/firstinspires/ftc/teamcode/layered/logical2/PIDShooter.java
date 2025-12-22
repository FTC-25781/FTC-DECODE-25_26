package org.firstinspires.ftc.teamcode.layered.logical2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.layered.control3.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.layered.physical1.FlywheelMotor;
@TeleOp(name="Angular PID")
public class PIDShooter extends LinearOpMode {

    public Servo servo;
    public FlywheelMotor flywheel;
    public GetDistanceToGoal distanceCalculation;
    public Pose startingPose = new Pose(72, 72, Math.toRadians(135));
    public Follower follower;

    @Override
    public void runOpMode() {
        initHardware();

        waitForStart();
        servo.setPosition(0.5);

        while (opModeIsActive()) {
            follower.update();
            // flywheel.updatePIDFCoefficients();

            double distance = distanceCalculation.getDistanceToGoal();
            double targetRPM = flywheel.targetRPM(distance);
            flywheel.setRPM(targetRPM);

            if(gamepad1.a){
                servo.setPosition(0.63);
            }
            if(gamepad1.b){
                servo.setPosition(0.05);
            }
            telemetry.addData("Current Position-x:", follower.getPose().getX());
            telemetry.addData("Current Position-Y:", follower.getPose().getY());
            telemetry.addData("Current Heading:", follower.getPose().getHeading());
            telemetry.addData("Distance", distance);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", flywheel.getCurrentRPM());
            telemetry.addData("Velocity (rad/s)", flywheel.flywheelShooter.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("Motor Power", flywheel.flywheelShooter.getPower());
            telemetry.addData("Ready", flywheel.isShooterReady());
            telemetry.update();
        }
    }
    public void initHardware() {
        servo = hardwareMap.get(Servo.class, "liftServo");
        flywheel = new FlywheelMotor(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        distanceCalculation = new GetDistanceToGoal(follower, true);
    }
}