package org.firstinspires.ftc.teamcode.layeredFinal.control;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartIntakeMotor;

public class Intake {
    SmartIntakeMotor intake;

    public Intake(HardwareMap hardwareMap) {
        intake = new SmartIntakeMotor(hardwareMap);
    }

    public void forward() {
        intake.startRotation();
    }

    public void stopped() {
        intake.stopRotation();
    }

    public void reverse() {
        intake.reverseRotation();
    }
}
