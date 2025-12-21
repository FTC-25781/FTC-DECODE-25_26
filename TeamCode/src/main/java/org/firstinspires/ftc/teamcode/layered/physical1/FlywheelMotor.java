package org.firstinspires.ftc.teamcode.layered.physical1;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.layered.logical2.BallisticsCalculator;


// This class is for defining all the motor functions that will be used for the flywheel.
/*
Functionalities:
1. Initializing the hardware
2. Setting the velocity of the flywheel for RPM
 */
@Config
public class FlywheelMotor{
    public DcMotorEx flywheelShooter;
    Telemetry telemetryF;
    //public static final double TICKS_PER_REV = 28.0;
    public static final double RPM_TOLERANCE = 50.0;
    public double targetRPM;
    public static double kP = 10.0;
    public static double kI = 0.1;
    public static double kD = 5.0;
    public static double kF = 12.0;
    public FlywheelMotor(HardwareMap hardwareMap, Telemetry telemetry) {
        flywheelShooter = hardwareMap.get(DcMotorEx.class, "dmot");
        telemetryF = telemetry;
        flywheelShooter.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheelShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        updatePIDFCoefficients();
    }
    public void updatePIDFCoefficients() {
        flywheelShooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }
    public void setRPM(double targetRPM){
        this.targetRPM = targetRPM;
        double angularVel = RPMToAngular(targetRPM);
        flywheelShooter.setVelocity(angularVel);

        telemetryF.addData("Angular Velocity", angularVel);
    }
    public boolean isShooterReady() {
        return Math.abs(targetRPM - angularToRPM(flywheelShooter.getVelocity(AngleUnit.RADIANS))) <
                RPM_TOLERANCE;
    }

    public double targetRPM(double distance) {
        double calculatedRPM = BallisticsCalculator.calculateRequiredRPM(distance);

        return Range.clip(calculatedRPM, 0, 6000);
    }
    private double angularToRPM(double radPerSec){
        return radPerSec * 60 / (2 * Math.PI);
    }
    private double RPMToAngular(double RPM){
        double angularVelocity = RPM * (2 * Math.PI) / 60;
        return angularVelocity;
    }
    public double getCurrentRPM() {
        return angularToRPM(flywheelShooter.getVelocity(AngleUnit.RADIANS));
    }
}
