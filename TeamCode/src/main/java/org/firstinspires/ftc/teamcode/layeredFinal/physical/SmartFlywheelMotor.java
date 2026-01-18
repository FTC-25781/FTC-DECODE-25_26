package org.firstinspires.ftc.teamcode.layeredFinal.physical;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class SmartFlywheelMotor {
    private DcMotorEx flywheelMotor;
    double currTargetVelocity = 0;

    final double P = 344;
    final double F = 14;

    public SmartFlywheelMotor(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "dmot");
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorEx.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void update() {
        flywheelMotor.setVelocity(currTargetVelocity);
    }

    public void setVelocity(double vel) {
        currTargetVelocity = vel;
    }

    public double getCurVelocity() {
        return flywheelMotor.getVelocity();
    }

    public double getError() {
        return currTargetVelocity - getCurVelocity();
    }
}
