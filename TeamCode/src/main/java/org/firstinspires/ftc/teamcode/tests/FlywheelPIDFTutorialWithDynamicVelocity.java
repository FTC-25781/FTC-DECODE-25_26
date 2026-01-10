package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@TeleOp
public class FlywheelPIDFTutorialWithDynamicVelocity extends OpMode {
    public DcMotorEx flywheelMotor;

    Follower follower;
    public boolean isBlue;
    public Pose startingPose = new Pose(72, 72, Math.toRadians(135));

    double currTargetVelocity = 0;  // will be computed dynamically
    double P = 342; // Makes it regain velocity super fast (0.2 - 0.5 sec wait)
    double F = 14;  // Initial velocity comes from feedforward

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001}; // Aids in testing
    int stepIndex = 1;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "dmot");
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorEx.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Is Brogan Cooking?"); // PS. he cooked

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
    }

    @Override
    public void loop() {
        follower.update();
        double distanceMM = getDistanceToTargetMM();
        currTargetVelocity = computeFlywheelVelocityMM(distanceMM);

        currTargetVelocity *= 2;

        // PIDF tuning via gamepad (Not needed anymore)
        if (gamepad1.bWasPressed()) { stepIndex = (stepIndex + 1) % stepSizes.length; }
        if (gamepad1.dpadLeftWasPressed()) F -= stepSizes[stepIndex];
        if (gamepad1.dpadRightWasPressed()) F += stepSizes[stepIndex];
        if (gamepad1.dpadDownWasPressed()) P -= stepSizes[stepIndex];
        if (gamepad1.dpadUpWasPressed()) P += stepSizes[stepIndex];

        // Update PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Set flywheel velocity
        flywheelMotor.setVelocity(currTargetVelocity);

        // Telemetry
        double curVelocity = flywheelMotor.getVelocity();
        double error = currTargetVelocity - curVelocity;
        telemetry.addData("Distance (mm)", "%.2f", distanceMM);
        telemetry.addData("Target Velocity", "%.2f", currTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }

    private double getDistanceToTargetMM() {
        Pose robotPose = this.follower.getPose();
        double goalX;

        if(isBlue) {
            goalX = 12; // Blue
        } else {
            goalX = 132; // Red
        }

        double goalY = 132; // Constant
        double dx = goalX - robotPose.getX();
        double dy = goalY - robotPose.getY();

        return (Math.sqrt(dx * dx + dy * dy) * 25.4);
    }

    // ChatGPT Formula
    public double computeFlywheelVelocityMM(double distanceMM) {
        double x = distanceMM; // horizontal distance in mm
        double shooterHeight = 333.8;  // mm
        double targetHeight  = 1092.2; // mm
        double y = targetHeight - shooterHeight;

        double theta = Math.toRadians(35); // shooter angle
        double g = 9810; // mm/s²

        double vExit = Math.sqrt(
                (g * x * x) /
                        (2 * Math.pow(Math.cos(theta), 2) * (x * Math.tan(theta) - y))
        );

        double flywheelRadius = 48.0; // mm
        double rpm = (vExit / (2 * Math.PI * flywheelRadius)) * 60.0;

        double ticksPerRev = 28;
        return rpm * ticksPerRev / 60.0; // motor velocity in ticks/sec
    }
}
