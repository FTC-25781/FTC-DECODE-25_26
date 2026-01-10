package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "MiniTransferTestServo", group = "tests")
public class MiniTransferTestServo extends LinearOpMode {
    private Servo servo;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(0.1);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.dpad_up) {
                servo.setPosition(servo.getPosition() + 0.1);
            }

            if (gamepad1.dpad_down) {
                servo.setPosition(servo.getPosition() - 0.1);
            }

            telemetry.addData("Servo Pos", servo.getPosition());
            telemetry.update();
        }
    }
}
