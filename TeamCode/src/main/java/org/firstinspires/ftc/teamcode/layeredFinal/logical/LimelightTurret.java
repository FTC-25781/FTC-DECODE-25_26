package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class LimelightTurret{

    DcMotorEx turretMotor;
    Limelight limelight;
    private double kP = 0.027;
    private double kD = 0.0009;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.2;
    private final ElapsedTime timer = new ElapsedTime();
    public double power = 0;

    public LimelightTurret(HardwareMap hardwareMap){
        this.limelight = new Limelight(hardwareMap);
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetTimer(){
        timer.reset();
    }
    public void update(){
        double deltaTime = timer.seconds();
        timer.reset();

        if(limelight.getID() == 0){
            turretMotor.setPower(0);
            lastError = 0;
            return;
        }
        double error = goalX - limelight.getAprilTagTargetX();
        double pTerm = error * kP;
        double dTerm = 0;
        if(deltaTime > 0){
            dTerm = ((error - lastError) / deltaTime) * kD;
        }
        if(Math.abs(error) < angleTolerance){
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -1, 1);
        }
        turretMotor.setPower(power);
        lastError = error;
    }
}
