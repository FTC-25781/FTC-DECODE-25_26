package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.colorSensorDriver.colorSensorDriver;

@TeleOp(name = "Color Sensor Test", group = "Test")
public class ColorSensorForTransfer extends LinearOpMode {
    private colorSensorDriver color1, color2, color3;

    @Override
    public void runOpMode() {
        color1 = hardwareMap.get(colorSensorDriver.class, "Color1");
        color2 = hardwareMap.get(colorSensorDriver.class, "Color2");
        color3 = hardwareMap.get(colorSensorDriver.class, "Color3");

        color1.tuneVals(475, 500, 280, 560, 570, 332);
        color2.tuneVals(2245,2653,1577,6200,6900, 4415);
        color3.tuneVals(231, 277, 155, 500, 560, 325);

        telemetry.addLine("Initialized. Ready to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            color1.update();
            color2.update();
            color3.update();

            telemetry.addData("Kicker 1", detectColor1(color1));
            telemetry.addData("Kicker 2", detectColor2And3(color2));
            telemetry.addData("Kicker 3", detectColor2And3(color3));

            telemetry.addLine("--- Data (Sensor 1) ---");
            displayDebug(color1);
            telemetry.addLine("--- Data (Sensor 2) ---");
            displayDebug(color2);
            telemetry.addLine("--- Data (Sensor 3) ---");
            displayDebug(color3);

            telemetry.update();
        }
    }

    private String detectColor1(colorSensorDriver sensor) {
        if ((sensor.green - sensor.red) >= 40) {
            return "green";
        } else if ((sensor.green - sensor.red) <= 20) {
            return "purple";
        } else {
            return "not detected";
        }
    }

    private String detectColor2And3(colorSensorDriver sensor) {
        if ((sensor.red - sensor.green) >= 2) {
            return "purple";
        } else if ((sensor.red - sensor.green) <= -2) {
            return "green";
        } else {
            return "not detected";
        }
    }

    private void displayDebug(colorSensorDriver sensor) {
        telemetry.addData("Clear", sensor.rclear);
        telemetry.addData("Red", sensor.rred);
        telemetry.addData("Green", sensor.rgreen);
        telemetry.addData("Blue", sensor.rblue);
        telemetry.addLine();
        telemetry.addData("N. Clear", sensor.clear);
        telemetry.addData("N. Red", sensor.red);
        telemetry.addData("N. Green", sensor.green);
        telemetry.addData("N. Blue", sensor.blue);
    }
}
