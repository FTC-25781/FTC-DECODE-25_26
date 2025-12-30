package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartColorSensor;

public class TransferColorSensor {
    SmartColorSensor colorSensors;

    public TransferColorSensor(HardwareMap hardwareMap) {
        colorSensors = new SmartColorSensor(hardwareMap);
    }

    public int colorOfSensor1() {
        if (colorSensors.getGreen1() > colorSensors.getBlue1() && colorSensors.getRawClear1() > 160) {
            return 1; // green
        } else if (colorSensors.getGreen1() < colorSensors.getBlue1() && colorSensors.getRawClear1() > 160) {
            return 2; // purple
        } else {
            return 0; // nothing
        }
    }

    public int colorOfSensor2() {
        if (colorSensors.getGreen2() > colorSensors.getBlue2() && colorSensors.getRawClear2() > 160) {
            return 1; // green
        } else if (colorSensors.getGreen2() < colorSensors.getBlue2() && colorSensors.getRawClear2() > 160) {
            return 2; // purple
        } else {
            return 0; // nothing
        }
    }

    public int colorOfSensor3() {
        if (colorSensors.getGreen3() > colorSensors.getBlue3() && colorSensors.getRawClear3() > 160) {
            return 1; // green
        } else if (colorSensors.getGreen3() < colorSensors.getBlue3() && colorSensors.getRawClear3() > 160) {
            return 2; // purple
        } else {
            return 0; // nothing
        }
    }

    public void update() {
        colorSensors.update();
    }
}
