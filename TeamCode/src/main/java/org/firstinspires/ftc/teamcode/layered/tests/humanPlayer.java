package org.firstinspires.ftc.teamcode.layered.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp(name = "Human Player", group = "test code")
public class humanPlayer extends LinearOpMode {
    private DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "dmot");

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(-gamepad1.left_trigger);

            if (gamepad1.x) {
                motor.setPower(0.5);
            }

            if (gamepad1.a) {
                motor.setPower(-0.5);
            }

            if (gamepad1.b) {
                motor.setPower(0);
            }

            telemetry.addData("Motor Power", motor.getPower());
            telemetry.update();
        }
    }
}
