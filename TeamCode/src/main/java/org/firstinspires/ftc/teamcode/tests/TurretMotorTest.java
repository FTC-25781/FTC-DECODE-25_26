package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "ARYAN RUN THIS", group = "test")
public class TurretMotorTest extends OpMode {
    DcMotorEx motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class,"tmot");
    }

    @Override
    public void loop() {
        motor.setPower(0.75);

        if (gamepad1.aWasPressed()) {
            motor.setPower(0);
        }
        if (gamepad1.bWasPressed()) {
            motor.setPower(1);
        }
    }
}
