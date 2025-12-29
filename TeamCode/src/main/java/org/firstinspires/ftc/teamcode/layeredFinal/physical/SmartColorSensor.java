package org.firstinspires.ftc.teamcode.layeredFinal.physical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.colorSensorDriver.colorSensorDriver;

public class SmartColorSensor {
    colorSensorDriver colorSensor;

    public SmartColorSensor(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(colorSensorDriver.class, "sensorColor");
    }

    public double getRawClear() {
        return colorSensor.rclear;
    }

    public double getRawRed() {
        return colorSensor.rred;
    }

    public double getRawBlue() {
        return colorSensor.rblue;
    }

    public double getRawGreen() {
        return colorSensor.rgreen;
    }

    public double getClear() {
        return colorSensor.clear;
    }

    public double getRed() {
        return colorSensor.red;
    }

    public double getBlue() {
        return colorSensor.blue;
    }

    public double getGreen() {
        return colorSensor.green;
    }

    // TODO: MOVE TO LOGICAL
    public boolean isGreenBall() {
        return getGreen() > getBlue() && getRawClear() > 160;
    }

    // TODO: MOVE TO LOGICAL
    public boolean isPurpleBall() {
        return getGreen() < getBlue() && getRawClear() > 160;
    }

    public void update() {
        colorSensor.update();
    }
}
