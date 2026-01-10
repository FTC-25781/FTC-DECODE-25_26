package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartColorSensor;

public class TransferColorSensor {
    public enum DetectedColor {
        NONE,   // Empty slot (High clear value)
        GREEN,  // Green game element detected
        PURPLE  // Purple game element detected
    }

    private static final int CLEAR_THRESHOLD = 500;
    private static final double GREEN_BIAS_MULTIPLIER = 1.2;
    private final SmartColorSensor colorSensors;

    public TransferColorSensor(HardwareMap hardwareMap) {
        colorSensors = new SmartColorSensor(hardwareMap);
    }

    private DetectedColor detectColor(double green, double blue, double clear) {
        if (clear <= CLEAR_THRESHOLD) {
            double greenRatio = green / clear;
            double blueRatio = blue / clear;

            if (greenRatio > (blueRatio * GREEN_BIAS_MULTIPLIER)) {
                return DetectedColor.GREEN;
            } else {
                return DetectedColor.PURPLE;
            }
        }

        return DetectedColor.NONE;
    }

    public DetectedColor colorOfSensor1() {
        return detectColor(
                colorSensors.getGreen1(),
                colorSensors.getBlue1(),
                colorSensors.getRawClear1()
        );
    }

    public DetectedColor colorOfSensor2() {
        return detectColor(
                colorSensors.getGreen2(),
                colorSensors.getBlue2(),
                colorSensors.getRawClear2()
        );
    }

    public DetectedColor colorOfSensor3() {
        return detectColor(
                colorSensors.getGreen3(),
                colorSensors.getBlue3(),
                colorSensors.getRawClear3()
        );
    }

    public void update() {
        colorSensors.update();
    }
}
