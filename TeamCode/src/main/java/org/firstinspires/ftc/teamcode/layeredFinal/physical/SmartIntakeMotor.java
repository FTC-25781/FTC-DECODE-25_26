package org.firstinspires.ftc.teamcode.layeredFinal.physical;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SmartIntakeMotor {
    private DcMotorEx intakeMotor;

    public SmartIntakeMotor(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "imot");
    }

    public void startRotation() { // Start rotation forward
        intakeMotor.setPower(1);
    }

    public void stopRotation() { // Stop the rotation
        intakeMotor.setPower(0);
    }

    public void reverseRotation() { // Reverse in-case ball gets stuck
        intakeMotor.setPower(-1);
    }
}
