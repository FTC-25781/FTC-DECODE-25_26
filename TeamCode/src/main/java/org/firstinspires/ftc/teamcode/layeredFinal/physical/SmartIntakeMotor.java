package org.firstinspires.ftc.teamcode.layeredFinal.physical;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SmartIntakeMotor {
    private DcMotorEx intakeMotor;

    public SmartIntakeMotor(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "imot");
    }

    public void startRotation() {
        intakeMotor.setPower(1);
    }

    public void stopRotation() {
        intakeMotor.setPower(0);
    }

    public void reverseRotation() {
        intakeMotor.setPower(-1);
    }
}
