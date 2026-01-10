package org.firstinspires.ftc.teamcode.layeredFinal.physical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.colorSensorDriver.colorSensorDriver;

public class SmartColorSensor {
    // Custom driver instances for each physical sensor
    colorSensorDriver colorSensor1;
    colorSensorDriver colorSensor2;
    colorSensorDriver colorSensor3;

    public SmartColorSensor(HardwareMap hardwareMap) {
        // These strings ("Color1", etc.) must match the names saved on the Expansion/Control Hub config
        colorSensor1 = hardwareMap.get(colorSensorDriver.class, "Color1");
        colorSensor2 = hardwareMap.get(colorSensorDriver.class, "Color2");
        colorSensor3 = hardwareMap.get(colorSensorDriver.class, "Color3");
    }

    // --- Sensor 1 Getters ---
    public double getRawClear1() { return colorSensor1.rclear; }
    public double getBlue1() { return colorSensor1.blue; }
    public double getGreen1() { return colorSensor1.green; }

    // --- Sensor 2 Getters ---
    public double getRawClear2() { return colorSensor2.rclear; }
    public double getBlue2() { return colorSensor2.blue; }
    public double getGreen2() { return colorSensor2.green; }

    // --- Sensor 3 Getters ---
    public double getRawClear3() { return colorSensor3.rclear; }
    public double getBlue3() { return colorSensor3.blue; }
    public double getGreen3() { return colorSensor3.green; }

    public void update() {
        colorSensor1.update();
        colorSensor2.update();
        colorSensor3.update();
    }
}
