package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartColorSensor;

public class TransferColorSensor {
    SmartColorSensor colorSensor;

    public TransferColorSensor(HardwareMap hardwareMap) {
        colorSensor = new SmartColorSensor(hardwareMap);
    }

    public boolean isGreenBall() {
        return colorSensor.getGreen() > colorSensor.getBlue() && colorSensor.getRawClear() > 160;
    }

    public boolean isPurpleBall() {
        return colorSensor.getGreen() < colorSensor.getBlue() && colorSensor.getRawClear() > 160;
    }
}
