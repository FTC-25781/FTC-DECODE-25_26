package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartColorSensor;

/**
 * Interprets raw data from the robot's three color sensors.
 * It abstracts the light frequency data into easy-to-use "Green", "Purple", or "None" states.
 */
public class TransferColorSensor {

    // Simplified classification of what the sensors can see
    public enum DetectedColor {
        NONE,   // No object present or light too dim
        GREEN,  // Object identified as Green
        PURPLE  // Object identified as Purple
    }

    // Minimum light intensity required to confirm an object is present.
    // Prevents "ghost" readings when the sensor is looking at empty space.
    private static final int CLEAR_THRESHOLD = 160;

    // Reference to the physical sensor hardware wrapper
    private final SmartColorSensor colorSensors;

    public TransferColorSensor(HardwareMap hardwareMap) {
        colorSensors = new SmartColorSensor(hardwareMap);
    }

    /**
     * Logic to determine the color of a game element based on RGB ratios.
     * @param green Raw green channel value
     * @param blue Raw blue channel value
     * @param clear Total light intensity (used for proximity/presence)
     * @return The classified DetectedColor
     */
    private DetectedColor detectColor(double green, double blue, double clear) {
        // If the sensor doesn't see enough light, assume the slot is empty
        if (clear <= CLEAR_THRESHOLD) {
            return DetectedColor.NONE;
        }

        // Compare color channels to distinguish between game elements.
        // Green elements will have a higher green value than blue.
        if (green > blue) {
            return DetectedColor.GREEN;
        }
        // Purple elements (often containing blue light) will have higher blue than green.
        else if (blue > green) {
            return DetectedColor.PURPLE;
        }

        return DetectedColor.NONE;
    }

    /**
     * @return The color detected at the first kicker position.
     */
    public DetectedColor colorOfSensor1() {
        return detectColor(
                colorSensors.getGreen1(),
                colorSensors.getBlue1(),
                colorSensors.getRawClear1()
        );
    }

    /**
     * @return The color detected at the second kicker position.
     */
    public DetectedColor colorOfSensor2() {
        return detectColor(
                colorSensors.getGreen2(),
                colorSensors.getBlue2(),
                colorSensors.getRawClear2()
        );
    }

    /**
     * @return The color detected at the third kicker position.
     */
    public DetectedColor colorOfSensor3() {
        return detectColor(
                colorSensors.getGreen3(),
                colorSensors.getBlue3(),
                colorSensors.getRawClear3()
        );
    }

    /**
     * Refreshes the hardware readings.
     * This should be called once per loop to ensure data is current.
     */
    public void update() {
        colorSensors.update();
    }
}
