package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartColorSensor;

/**
 * LOGICAL LAYER: TransferColorSensor
 * * This class acts as a translator between raw hardware data (RGB numbers) and
 * game-specific logic (Green vs Purple). It uses the successful logic derived
 * from the 'ColorSensorForTransfer' test OpMode.
 */
public class TransferColorSensor {

    /**
     * Represents the discrete states a sensor position can be in.
     */
    public enum DetectedColor {
        NONE,   // Empty slot (High clear value)
        GREEN,  // Green game element detected
        PURPLE  // Purple game element detected
    }

    /**v
     * CLEAR_THRESHOLD: The "Detection" trigger point.
     * In the test, a value BELOW this indicates an object is blocking/near the sensor.
     */
    private static final int CLEAR_THRESHOLD = 500;

    /**
     * GREEN_BIAS_MULTIPLIER: A sensitivity buffer.
     * Requires the Green ratio to be at least 20% higher than the Blue ratio
     * to be classified as "Green". This helps prevent "color flickering"
     * due to ambient light or slight variations in Purple elements.
     */
    private static final double GREEN_BIAS_MULTIPLIER = 1.2;

    // Internal reference to the hardware wrapper
    private final SmartColorSensor colorSensors;

    /**
     * Constructor: Initializes the hardware sensors via the physical layer.
     * @param hardwareMap The OpMode's hardware map.
     */
    public TransferColorSensor(HardwareMap hardwareMap) {
        colorSensors = new SmartColorSensor(hardwareMap);
    }

    /**
     * Core Logic: Processes raw RGB/Clear data into a DetectedColor state.
     * * Logic flow:
     * 1. Check if 'clear' is below the threshold (Object present).
     * 2. Normalize Green and Blue values against the Clear value (Ratios).
     * 3. Apply the bias multiplier to differentiate Green from Purple.
     */
    private DetectedColor detectColor(double green, double blue, double clear) {
        // Step 1: Presence Detection
        // If clear is above the threshold, the sensor sees 'open space' or 'white'
        if (clear <= CLEAR_THRESHOLD) {
            // Step 2: Normalize values
            // Dividing by 'clear' ensures that the logic works even if
            // the overall brightness changes slightly.
            double greenRatio = green / clear;
            double blueRatio = blue / clear;

            // Step 3: Classification
            if (greenRatio > (blueRatio * GREEN_BIAS_MULTIPLIER)) {
                return DetectedColor.GREEN;
            } else {
                // If an object is present but doesn't meet the Green criteria, it's Purple.
                return DetectedColor.PURPLE;
            }
        }

        // Default state when no object is close enough to the sensor.
        return DetectedColor.NONE;
    }

    /**
     * @return Current color state for Sensor 1 (e.g., Kicker 1).
     */
    public DetectedColor colorOfSensor1() {
        return detectColor(
                colorSensors.getGreen1(),
                colorSensors.getBlue1(),
                colorSensors.getRawClear1()
        );
    }

    /**
     * @return Current color state for Sensor 2 (e.g., Kicker 2).
     */
    public DetectedColor colorOfSensor2() {
        return detectColor(
                colorSensors.getGreen2(),
                colorSensors.getBlue2(),
                colorSensors.getRawClear2()
        );
    }

    /**
     * @return Current color state for Sensor 3 (e.g., Kicker 3).
     */
    public DetectedColor colorOfSensor3() {
        return detectColor(
                colorSensors.getGreen3(),
                colorSensors.getBlue3(),
                colorSensors.getRawClear3()
        );
    }

    /**
     * Hardware Refresh: Pulls the latest data from the physical sensors.
     * This should be called once at the start of every TeleOp/Auto loop.
     */
    public void update() {
        colorSensors.update();
    }
}
