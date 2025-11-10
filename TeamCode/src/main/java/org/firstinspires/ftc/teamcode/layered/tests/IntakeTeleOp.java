package org.firstinspires.ftc.teamcode.layered.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.layered.control3.SorterServoSubsystem;

@TeleOp(name = "test intake", group = "tests")
public class IntakeTeleOp extends LinearOpMode {
    private SorterServoSubsystem servo;

    @Override
    public void runOpMode() {
        servo = new SorterServoSubsystem(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                servo.start();
            }

            servo.update();
        }
    }
}
