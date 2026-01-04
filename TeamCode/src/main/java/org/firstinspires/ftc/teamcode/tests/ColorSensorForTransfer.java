package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.colorSensorDriver.colorSensorDriver;

@TeleOp(name = "Color Sensor: Normalized Logic", group = "Test")
public class ColorSensorForTransfer extends LinearOpMode {

    private colorSensorDriver color1, color2, color3;
    private final int CLEAR_THRESHOLD = 500;

    @Override
    public void runOpMode() {
        color1 = hardwareMap.get(colorSensorDriver.class, "color1");
        color2 = hardwareMap.get(colorSensorDriver.class, "color2");
        color3 = hardwareMap.get(colorSensorDriver.class, "color3");

        telemetry.addLine("Initialized. Ready to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            color1.update();
            color2.update();
            color3.update();

            telemetry.addData("Kicker 1", detectColor(color1));
            telemetry.addData("Kicker 2", detectColor(color2));
            telemetry.addData("Kicker 3", detectColor(color3));

            telemetry.addLine("--- Raw Data (Sensor 1) ---");
            displayDebug(color1);

            telemetry.update();
        }
    }

    private String detectColor(colorSensorDriver sensor) {
        if (sensor.rclear < CLEAR_THRESHOLD) {
            double greenRatio = (double) sensor.rgreen / sensor.rclear;
            double blueRatio = (double) sensor.rblue / sensor.rclear;

            if (greenRatio > (blueRatio * 1.2)) {
                return "GREEN Ball";
            } else {
                return "PURPLE Ball";
            }
        } else {
            return "No Ball";
        }
    }

    private void displayDebug(colorSensorDriver sensor) {
        telemetry.addData("Clear", sensor.rclear);
        telemetry.addData("Red", sensor.rred);
        telemetry.addData("Green", sensor.rgreen);
        telemetry.addData("Blue", sensor.rblue);
    }
}
