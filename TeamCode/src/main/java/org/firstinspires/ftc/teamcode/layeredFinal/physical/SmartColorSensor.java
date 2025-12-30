package org.firstinspires.ftc.teamcode.layeredFinal.physical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.colorSensorDriver.colorSensorDriver;

public class SmartColorSensor {
    colorSensorDriver colorSensor1;
    colorSensorDriver colorSensor2;
    colorSensorDriver colorSensor3;

    public SmartColorSensor(HardwareMap hardwareMap) {
        colorSensor1 = hardwareMap.get(colorSensorDriver.class, "Color1"); // TODO: Config on Driver Station
        colorSensor2 = hardwareMap.get(colorSensorDriver.class, "Color2");
        colorSensor3 = hardwareMap.get(colorSensorDriver.class, "Color3");
    }

    /* Getting the Clear / Blue / Green vals from each color sensor
       to be used to check if the ball is green or purple */
    public double getRawClear1() { return colorSensor1.rclear; }
    public double getBlue1() { return colorSensor1.blue; }
    public double getGreen1() { return colorSensor1.green; }

    public double getRawClear2() { return colorSensor2.rclear; }
    public double getBlue2() { return colorSensor2.blue; }
    public double getGreen2() { return colorSensor2.green; }

    public double getRawClear3() { return colorSensor3.rclear; }
    public double getBlue3() { return colorSensor3.blue; }
    public double getGreen3() { return colorSensor3.green; }


    public void update() { // Updating for latest vals
        colorSensor1.update();
        colorSensor2.update();
        colorSensor3.update();
    }
}
