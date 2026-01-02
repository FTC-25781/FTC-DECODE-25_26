package org.firstinspires.ftc.teamcode.layeredFinal.physical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.colorSensorDriver.colorSensorDriver;

/**
 * Physical layer wrapper for the three color sensors.
 * This class handles direct hardware communication and ensures the Driver Station
 * configuration names match the code.
 */
// TODO: Need to calibrate and test
public class SmartColorSensor {
    // Custom driver instances for each physical sensor
    colorSensorDriver colorSensor1;
    colorSensorDriver colorSensor2;
    colorSensorDriver colorSensor3;

    /**
     * Maps the software objects to the physical ports defined on the Robot Configuration.
     * @param hardwareMap The hardware map provided by the OpMode.
     */
    public SmartColorSensor(HardwareMap hardwareMap) {
        // These strings ("Color1", etc.) must match the names saved on the Expansion/Control Hub config
        colorSensor1 = hardwareMap.get(colorSensorDriver.class, "Color1");
        colorSensor2 = hardwareMap.get(colorSensorDriver.class, "Color2");
        colorSensor3 = hardwareMap.get(colorSensorDriver.class, "Color3");
    }

    /* * The following methods provide raw light data (Clear, Blue, Green).
     * rclear: Total light intensity (useful for detecting if an object is present).
     * blue/green: Filtered light levels (useful for identifying the game element's color).
     */

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


    /**
     * Refreshes the hardware registers for all sensors.
     * Because I2C communication is relatively slow, calling this once per loop
     * is more efficient than reading sensors individually multiple times.
     */
    public void update() {
        colorSensor1.update();
        colorSensor2.update();
        colorSensor3.update();
    }
}
