package org.firstinspires.ftc.teamcode.layeredOld.physical1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EncoderForIntake {
    private DcMotorEx encoder;
    private int target = 800;

    public EncoderForIntake(HardwareMap hardwareMap) {
        encoder = hardwareMap.get(DcMotorEx.class, "encoder");
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void resetEncoder() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isAtTarget() {
        return Math.abs(encoder.getCurrentPosition()) >= target;
    }

    public float pos() {
        return encoder.getCurrentPosition();
    }
}
