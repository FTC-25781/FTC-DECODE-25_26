package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartColorSensor;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.colorSensorDriver.colorSensorDriver;

public class TransferColorSensor {
    public enum DetectedColor {
        NONE,
        GREEN,
        PURPLE
    }

    private final SmartColorSensor colorSensors;

    /***
     *  Constructor for color sensor
      * @param hardwareMap
     */
    public TransferColorSensor(HardwareMap hardwareMap) {
        colorSensors = new SmartColorSensor(hardwareMap);
    }

    /***
     * Detect color for the first sensor
     * @param red
     * @param green
     * @return color of the ball
     */
    private DetectedColor detectColor1(double red, double green) {
        if ((green - red) >= 40) {
            return DetectedColor.GREEN;
        } else if ((green - red) <= 20) {
            return DetectedColor.PURPLE;
        } else {
            return DetectedColor.NONE;
        }
    }

    /***
     * Detect color for second and third sensor
     * @param red
     * @param green
     * @return
     */
    private DetectedColor detectColor2And3(double red, double green) {
        if ((red - green) >= 2) {
            return DetectedColor.PURPLE;
        } else if ((red - green) <= -2) {
            return DetectedColor.GREEN;
        } else {
            return DetectedColor.NONE;
        }
    }

    /***
     *
     * @return
     */
    public DetectedColor colorOfSensor1() {
        return detectColor1(
                colorSensors.getRed1(),
                colorSensors.getGreen1()
        );
    }

    public DetectedColor colorOfSensor2() {
        return detectColor2And3(
                colorSensors.getRed2(),
                colorSensors.getGreen2()
        );
    }

    public DetectedColor colorOfSensor3() {
        return detectColor2And3(
                colorSensors.getRed3(),
                colorSensors.getGreen3()
        );
    }

    public void update() {
        colorSensors.update();
    }
}
