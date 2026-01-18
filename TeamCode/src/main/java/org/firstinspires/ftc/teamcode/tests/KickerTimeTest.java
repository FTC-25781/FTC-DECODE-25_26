package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.layeredFinal.control.Transfer;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.Flywheel;

@TeleOp
public class KickerTimeTest extends OpMode {
    Transfer t;
    Flywheel f;

    @Override
    public void init() {
        t = new Transfer(hardwareMap);
        f = new Flywheel(hardwareMap);

        t.update();
        f.update();
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            t.kicker1_time_value += 0.01;
        }

        if (gamepad1.aWasPressed()) {
            t.kicker1_time_value -= 0.01;
        }

        if (gamepad1.xWasPressed()) {
            t.kicker1_servo_lower_time -= 0.01;
        }

        if (gamepad1.bWasPressed()) {
            t.kicker1_servo_lower_time += 0.01;
        }

        // ---------------------------------

        if (gamepad1.dpadUpWasPressed()) {
            t.kicker2_time_value += 0.01;
        }

        if (gamepad1.dpadDownWasPressed()) {
            t.kicker2_time_value -= 0.01;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            t.kicker2_servo_lower_time -= 0.01;
        }

        if (gamepad1.dpadRightWasPressed()) {
            t.kicker2_servo_lower_time += 0.01;
        }

        // ------------------------------------

        if (gamepad1.rightBumperWasPressed()) {
            t.kicker3_time_value += 0.01;
        }

        if (gamepad1.leftBumperWasPressed()) {
            t.kicker3_time_value -= 0.01;
        }

        if (gamepad1.leftStickButtonWasPressed()) {
            t.kicker3_servo_lower_time -= 0.01;
        }

        if (gamepad1.rightStickButtonWasPressed()) {
            t.kicker3_servo_lower_time += 0.01;
        }

        if (gamepad1.startWasPressed()) {
            t.startKickSequenceRandomly();
        }

        if (gamepad2.aWasPressed()) {
            f.setVelForFarTip();
        }

        if (gamepad2.bWasPressed()) {
            f.stopFlywheel();
        }

        if (gamepad2.dpadUpWasPressed()) {
            f.highVelocity += 10;
        }

        if (gamepad2.dpadDownWasPressed()) {
            f.highVelocity -= 10;
        }

        t.update();
        f.update();

        telemetry.addLine("Kicker 1:");
        telemetry.addData("raise", t.kicker1_time_value);
        telemetry.addData("lower", t.kicker1_servo_lower_time);
        telemetry.addLine("Kicker 2:");
        telemetry.addData("raise", t.kicker2_time_value);
        telemetry.addData("lower", t.kicker2_servo_lower_time);
        telemetry.addLine("Kicker 3:");
        telemetry.addData("raise", t.kicker3_time_value);
        telemetry.addData("lower", t.kicker3_servo_lower_time);
        telemetry.update();
    }
}
