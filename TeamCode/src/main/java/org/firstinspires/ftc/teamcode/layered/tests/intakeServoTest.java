package org.firstinspires.ftc.teamcode.layered.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "intakeServoTest", group = "tests")
public class intakeServoTest extends LinearOpMode {

    private CRServo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(CRServo.class, "transfer1");

        waitForStart();

        while (opModeIsActive()) {
            servo.setPower(gamepad1.left_stick_y);
        }
    }
}
