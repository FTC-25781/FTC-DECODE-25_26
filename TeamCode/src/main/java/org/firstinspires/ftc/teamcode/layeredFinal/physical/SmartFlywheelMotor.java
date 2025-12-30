package org.firstinspires.ftc.teamcode.layeredFinal.physical;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class SmartFlywheelMotor {
    private DcMotorEx flywheelMotor;

    double currTargetVelocity = 0;

    // Tuned vals through the FlywheelPIDFTutorial.java file
    final double P = 342;
    final double F = 14;

    public SmartFlywheelMotor(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "dmot");
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Setting the PIDF coefficients to use the setVelocity() function
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void update() { // Setting with whatever velocity we need at the time
        flywheelMotor.setVelocity(currTargetVelocity);
    }

    public void setVelocity(double vel) { // Allows for manual velocity setting
        currTargetVelocity = vel;
    }

    public double getCurVelocity() { // Gets current velocity of motor for kickers
        return flywheelMotor.getVelocity();
    }

    public double getError() { // Allows us to check error for debugging
        return currTargetVelocity - getCurVelocity();
    }
}
