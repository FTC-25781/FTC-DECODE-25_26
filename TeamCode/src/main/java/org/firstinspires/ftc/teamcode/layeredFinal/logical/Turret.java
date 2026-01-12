package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Turret {
    public DcMotorEx turretMotor;
    public TurretTracker turretOrientation;
    public double rotationSpeed = 0.9; // max speed for turret
    public double angleTolerance = Math.toRadians(3);
    public double fastAdjustmentThreshold = Math.toRadians(20);
    public double kP = 0.07;
    private double lastError = 0;
    private double kD = 0.03;
    public Turret(HardwareMap hardwareMap, Follower follower) {
        this.turretOrientation = new TurretTracker(hardwareMap, follower);
        turretMotor = hardwareMap.get(DcMotorEx.class, "tmot");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Calculates the error of the turret to the goal.
     * Sets servo speed originally to 0
     * If the error is greater than 5 degrees then
     * servo speed is set  + or - 0.3 depending on rotational movement
     * if it has to move counter clockwise, -0.3 and vice versa
     */
    public void trackGoal(){
        double error = turretOrientation.calculateError();

        if(Math.abs(error) <= angleTolerance) {
            turretMotor.setPower(0);
            lastError = 0;
            return;
        }
        double derivative = error - lastError;
        lastError = error;
        double output;
        if(Math.abs(error) > fastAdjustmentThreshold){
            output = Math.signum(error) * rotationSpeed;
        }
        else{
            double proportional = kP * error - kD * derivative;
            output = Range.clip(proportional, -rotationSpeed, rotationSpeed);
        }
        turretMotor.setPower(output);
    }

    public boolean isOnTarget() {
        double error = turretOrientation.calculateError();
        return Math.abs(error) <= angleTolerance;
    }
}
