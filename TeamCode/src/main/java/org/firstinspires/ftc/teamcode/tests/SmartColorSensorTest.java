package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.layeredFinal.physical.colorSensorDriver.colorSensorDriver;

@Disabled
@TeleOp(name="TCS3472 Test-1", group="Sensor")
public class SmartColorSensorTest extends LinearOpMode {
    colorSensorDriver colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        colorSensor = hardwareMap.get(colorSensorDriver.class,"sensorColor");

        waitForStart();

        while (opModeIsActive())
        {
            colorSensor.update();
            telemetry.addData("raw_Clear",colorSensor.rclear);
            telemetry.addData("raw_Red",colorSensor.rred);
            telemetry.addData("raw_Blue",colorSensor.rblue);
            telemetry.addData("raw_Green",colorSensor.rgreen);
            telemetry.addData("Clear",colorSensor.clear);
            telemetry.addData("Red",colorSensor.red);
            telemetry.addData("Blue",colorSensor.blue);
            telemetry.addData("Green",colorSensor.green);
            if (colorSensor.green > colorSensor.blue && colorSensor.rclear>160) {
                telemetry.addLine("Green Ball");
            } else if (colorSensor.green < colorSensor.blue && colorSensor.rclear>160) {
                telemetry.addLine("Purple Ball");;
            } else {
                telemetry.addLine("No Ball");
            }

            telemetry.update();
        }
    }
}