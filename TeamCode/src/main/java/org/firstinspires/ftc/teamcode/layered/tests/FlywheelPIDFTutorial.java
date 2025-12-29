package org.firstinspires.ftc.teamcode.layered.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

// Brogan M. Pratt Video: https://www.youtube.com/watch?v=aPNCpZzCTKg
//This code works at tip of big triangle and works at tip of small triangle

@TeleOp
public class FlywheelPIDFTutorial extends OpMode {
    public DcMotorEx flywheelMotor;

    public double highVelocity = 1525; // tip of small triangle
    public double lowVelocity = 1400; // tip of big triangle

    double currTargetVelocity = highVelocity;
    double P = 342; // Makes it regain velocity super fast (0.2 - 0.5 sec wait)
    double F = 14; // Initial velocity comes from Feedforward

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
    }

    @Override
    public void loop() {
        // setup gamepad commands
        // set target vel
        // update telemetry

        if (gamepad1.yWasPressed()) { // RISING EDGE WAS ADDED IN RECENT UPDATE????????????
            if (currTargetVelocity == highVelocity) {
                currTargetVelocity = lowVelocity;
            } else { currTargetVelocity = highVelocity; }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        // set new PIDF Coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F); // Just updating everytime
        flywheelMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // set vel
        flywheelMotor.setVelocity(currTargetVelocity);

        double curVelocity = (flywheelMotor.getVelocity());
        double error = currTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", currTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);

        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}
