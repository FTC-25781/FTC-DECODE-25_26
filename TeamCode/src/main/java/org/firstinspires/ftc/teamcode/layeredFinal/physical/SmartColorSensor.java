package org.firstinspires.ftc.teamcode.layeredFinal.physical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.colorSensorDriver.colorSensorDriver;

public class SmartColorSensor {
    colorSensorDriver colorSensor1;
    colorSensorDriver colorSensor2;
    colorSensorDriver colorSensor3;

    public SmartColorSensor(HardwareMap hardwareMap) {
        colorSensor1 = hardwareMap.get(colorSensorDriver.class, "Color1");
        colorSensor2 = hardwareMap.get(colorSensorDriver.class, "Color2");
        colorSensor3 = hardwareMap.get(colorSensorDriver.class, "Color3");

        colorSensor1.tuneVals(475, 500, 280, 560, 570, 332);
        colorSensor2.tuneVals(2245,2653,1577,6200,6900, 4415);
        colorSensor3.tuneVals(231, 277, 155, 500, 560, 325);

    }

    // --- Sensor 1 Getters ---
    public double getRed1() { return colorSensor1.red; }
    public double getGreen1() { return colorSensor1.green; }

    // --- Sensor 2 Getters ---
    public double getRed2() { return colorSensor2.red; }
    public double getGreen2() { return colorSensor2.green; }

    // --- Sensor 3 Getters ---
    public double getRed3() { return colorSensor3.red; }
    public double getGreen3() { return colorSensor3.green; }

    public void update() {
        colorSensor1.update();
        colorSensor2.update();
        colorSensor3.update();
    }
}
