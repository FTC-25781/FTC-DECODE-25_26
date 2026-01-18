package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="Another Turret Test")
public class TurretTest2 extends OpMode {
    DcMotorEx encoder;
    static final int TICKS_PER_180_DEG = 171;
    static final int DEGREES_PER_180_TICKS = 180;
    static final int TICKS_PER_DEGREE = TICKS_PER_180_DEG / DEGREES_PER_180_TICKS;
    double degrees = 45;
    int targetTicks;

    public PIDFController turretPID;
    public double kP = 0.012;
    public double kI = 0.0;
    public double kD = 0.003; // 0.003
    public double kF = 0.0008;

    @Override
    public void init(){
        encoder = hardwareMap.get(DcMotorEx.class, "tmot");
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        encoder.setDirection(DcMotorEx.Direction.REVERSE);
        turretPID = new PIDFController(new PIDFCoefficients(kP, kI, kD, kF));
        targetTicks = encoder.getCurrentPosition() * TICKS_PER_DEGREE;



    }

    @Override
    public void loop(){
        if(gamepad1.dpadUpWasPressed()){
            degrees += 10;
        }
        if(gamepad1.dpadDownWasPressed()){
            degrees -= 10;
        }

        targetTicks = degreesToTicks(degrees);
        int currentTicks = encoder.getCurrentPosition();
        turretPID.setTargetPosition(targetTicks);
        turretPID.updatePosition(currentTicks);
        double pidOutput = turretPID.run();

        double power = Range.clip(pidOutput, -0.6, 0.6);
        encoder.setPower(power);
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Current Ticks", currentTicks);
        telemetry.addData("Degrees", degrees);
        telemetry.addData("Error", targetTicks - currentTicks);
    }

    private int degreesToTicks(double degrees) {
        int targetTicks = (int) Math.round(degrees * TICKS_PER_DEGREE);
        return targetTicks;
    }
}
