package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartFlywheelMotor;

public class Flywheel {
    SmartFlywheelMotor flywheel;

    public double highVelocity = 1525;
    public double lowVelocity = 1350;

    public Flywheel(HardwareMap hardwareMap) {
        flywheel = new SmartFlywheelMotor(hardwareMap);
    }

    public void setVelForCloseTip() {
        flywheel.setVelocity(lowVelocity);
    }

    public void setVelForFarTip() {
        flywheel.setVelocity(highVelocity);
    }

    public void updateHighVelocity(double amount) {
        highVelocity += amount;
    }

    public void updateLowVelocity(double amount) {
        lowVelocity += amount;
    }

    public void stopFlywheel() {
        flywheel.setVelocity(0.0);
    }

    public void setVelocity(double vel) {
        flywheel.setVelocity(vel);
    }

    public double getVelocity() {
        return flywheel.getCurVelocity();
    }

    public void humanPlayer() {
        // TODO: Change with real vel once tested
        flywheel.setVelocity(-10);
    }

    public void update() {
        flywheel.update();
    }
}
