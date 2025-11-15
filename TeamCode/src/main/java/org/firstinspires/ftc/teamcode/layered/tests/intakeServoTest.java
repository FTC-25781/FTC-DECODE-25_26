package org.firstinspires.ftc.teamcode.layered.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(name = "intakeServoTest", group = "tests")
public class intakeServoTest extends LinearOpMode {
    private Servo servo;
    boolean lasta = false;
    boolean lastb = false;
    boolean lasty = false;
    boolean lastdpd = false;
    boolean lastdpl = false;
    boolean lastdpr = false;
    boolean lastrt = false;
    boolean lastlt = false;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "transfer1");

        waitForStart();

        while (opModeIsActive()) {

            // Intake Pos
            if (gamepad1.a && !lasta) {
                servo.setPosition(0.16);
            }

            if (gamepad1.b && !lastb) {
                servo.setPosition(0.54);
            }

            if (gamepad1.y && !lasty) {
                servo.setPosition(0.9);
            }

            // Outtake Pos
            if (gamepad1.dpad_down && !lastdpd) {
                servo.setPosition(0.7);
            }

            if (gamepad1.dpad_left && !lastdpl) {
                servo.setPosition(0);
            }

            if (gamepad1.dpad_right && !lastdpr) {
                servo.setPosition(0.32);
            }

            if (gamepad1.left_bumper && !lastlt) {
                servo.setPosition(servo.getPosition() + 0.01);
            }

            if (gamepad1.right_bumper && !lastrt) {
                servo.setPosition(servo.getPosition() - 0.01);
            }

            lastlt = gamepad1.left_bumper;
            lastrt = gamepad1.right_bumper;

            lasta = gamepad1.a;
            lastb = gamepad1.b;
            lasty = gamepad1.y;

            lastdpd = gamepad1.dpad_down;
            lastdpl = gamepad1.dpad_left;
            lastdpr = gamepad1.dpad_right;

            telemetry.addData("Servo Pos", servo.getPosition());
            telemetry.update();
        }
    }
}
