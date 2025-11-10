package org.firstinspires.ftc.teamcode.layered.physical1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EncoderForIntake {
    private DcMotorEx encoder;
    private int targetPosition = 2400;

    public EncoderForIntake(HardwareMap hardwareMap) {
        encoder = hardwareMap.get(DcMotorEx.class, "encoder");
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isAtTarget() {
        return encoder.getCurrentPosition() >= (targetPosition - 10);
    }
}
