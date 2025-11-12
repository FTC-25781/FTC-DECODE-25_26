package org.firstinspires.ftc.teamcode.layered.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Encoder Test", group = "tests")
public class MotEncoder4Intake extends LinearOpMode {
    private DcMotorEx encoder;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        encoder = hardwareMap.get(DcMotorEx.class, "encoder");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Encoder Pos", "power (%.2f)", encoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
