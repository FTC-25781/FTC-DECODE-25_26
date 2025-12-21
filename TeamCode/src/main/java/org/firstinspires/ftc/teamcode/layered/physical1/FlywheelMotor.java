package org.firstinspires.ftc.teamcode.layered.physical1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// This class is for defining all the motor functions that will be used for the flywheel.
/*
Functionalities:
1. Initializing the hardware
2. Setting the velocity of the flywheel for RPM
 */
public class FlywheelMotor{
    public DcMotorEx flywheelShooter;
    Telemetry telemetryF;
    public static final double TICKS_PER_REV = 28.0;
    public static final double RPM_TOLERANCE = 50.0;
    public double targetRPM;

    public FlywheelMotor(HardwareMap hardwareMap, Telemetry telemetry) {
        flywheelShooter = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        telemetryF = telemetry;
        flywheelShooter.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheelShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }
    public void setRPM(double targetRPM){
        targetRPM = targetRPM;
        flywheelShooter.setVelocity(RPMToAngular(targetRPM));
    }

    public boolean isShooterReady() {
        return Math.abs(targetRPM - angularToRPM(flywheelShooter.getVelocity(AngleUnit.RADIANS))) <
                RPM_TOLERANCE;
    }
    public void update(){
        // boolean shooterReady = isShooterReady();
    }
    public double targetRPM(double distance){
        double x = distance;

        final double C4 = 7.13229E-9;
        final double C3 = -(0.00000244252);
        final double C2 = 0.000297598;
        final double C1 = -0.0129613;
        final double C0 = 0.819973;

        double targetRPM1 = (C4 * Math.pow(x, 4)) + (C3 * Math.pow(x, 3)) + (C2 * Math.pow(x, 2)) + (C1 * x) + C0;
        targetRPM1 = Range.clip(targetRPM1 * 6000, 0, 6000);
        return targetRPM1;
    }
    //REPLACE WITH BALLISTICS EQUATIONS
    private double angularToRPM(double radPerSec){
        radPerSec = flywheelShooter.getVelocity(AngleUnit.RADIANS);
        double actualRPM = radPerSec * 60 / (2 * Math.PI);
        return actualRPM;
    }
    private double RPMToAngular(double RPM){
        double angularVelocity = RPM * (2 * Math.PI) / 60;
        return angularVelocity;
    }
}
