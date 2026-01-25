package org.firstinspires.ftc.teamcode.layeredFinal.control;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartIntakeMotor;

public class Intake {
    SmartIntakeMotor intake;

    public Intake(HardwareMap hardwareMap) {
        intake = new SmartIntakeMotor(hardwareMap);
    }

    /***
     * Start intake motor
     */
    public void forward() {
        intake.startRotation();
    }
    /***
     * Stop intake motor
     */
    public void stopped() { // Really not needed but allows us to stop
        intake.stopRotation();
    }
    /***
     * Reverse intake motor
     */
    public void reverse() { // Really not needed but allows us to go reverse
        intake.reverseRotation();

    }
}
