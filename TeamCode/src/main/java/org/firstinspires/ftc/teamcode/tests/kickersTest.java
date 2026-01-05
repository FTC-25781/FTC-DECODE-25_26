package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Kickers Test", group = "Test")
public class kickersTest extends OpMode {
    private Servo kick1, kick2, kick3;

    private double pos1 = 0.0; //0.28
    private double pos2 = 0.0; //0.28
    private double pos3 = 0.0; //0.28

    private static final double STEP = 0.01;

    @Override
    public void init() {
        kick1 = hardwareMap.get(Servo.class, "kick1");
        kick2 = hardwareMap.get(Servo.class, "kick2");
        kick3 = hardwareMap.get(Servo.class, "kick3");

        kick3.setDirection(Servo.Direction.REVERSE);

        kick1.setPosition(pos1);
        kick2.setPosition(pos2);
        kick3.setPosition(pos3);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) {
            pos1 += STEP;
        }
        if (gamepad1.dpadDownWasPressed()) {
            pos1 -= STEP;
        }

        if (gamepad1.dpadRightWasPressed()) {
            pos2 += STEP;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            pos2 -= STEP;
        }

        if (gamepad1.yWasPressed()) {
            pos3 += STEP;
        }
        if (gamepad1.aWasPressed()) {
            pos3 -= STEP;
        }

        // Clamp to valid servo range
        pos1 = clamp(pos1);
        pos2 = clamp(pos2);
        pos3 = clamp(pos3);

        kick1.setPosition(pos1);
        kick2.setPosition(pos2);
        kick3.setPosition(pos3);

        // Telemetry
        telemetry.addData("Kick1 Pos", pos1);
        telemetry.addData("Kick2 Pos", pos2);
        telemetry.addData("Kick3 Pos", pos3);
        telemetry.addLine("STEP = " + STEP);
        telemetry.update();
    }

    private double clamp(double pos) {
        return Math.max(0.0, Math.min(1.0, pos));
    }
}
