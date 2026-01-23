package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartFlywheelMotor;

public class Flywheel {
    SmartFlywheelMotor flywheel;

    public double highVelocity = 1540;
    public double lowVelocity = 1330;

    public Flywheel(HardwareMap hardwareMap) {
        flywheel = new SmartFlywheelMotor(hardwareMap);
    }
    public void setVelForCloseTip() {
        flywheel.setVelocity(lowVelocity);
    }
    public void setVelForFarTip() {
        flywheel.setVelocity(highVelocity);
    }
    public void stopFlywheel() {
        flywheel.setVelocity(0);
    }

    //public void setVelocity(double vel) {
        //flywheel.setVelocity(vel);
    //}

    public double getVelocity() {
        return flywheel.getCurVelocity();
    }
    public void humanPlayer() { flywheel.setVelocity(-100); }

    public void update() {
        flywheel.update();
    }
}
