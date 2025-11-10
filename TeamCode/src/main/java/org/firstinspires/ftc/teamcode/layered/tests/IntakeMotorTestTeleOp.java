package org.firstinspires.ftc.teamcode.layered.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.layered.control3.SorterServoSubsystem;
import org.firstinspires.ftc.teamcode.layered.physical1.IntakeMotor;

@TeleOp(name = "test motor intake", group = "tests")
public class IntakeMotorTestTeleOp extends LinearOpMode {
    private IntakeMotor mot;
    private SorterServoSubsystem servo;

    @Override
    public void runOpMode() {
        mot = new IntakeMotor(hardwareMap);
        servo = new SorterServoSubsystem(hardwareMap);

        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                mot.startIntaking();
            }

            if (gamepad1.b) {
                mot.returnToIdle();
            }

            if (gamepad1.y) {
                servo.start();
            }

            mot.update();
            servo.update();
        }
    }
}
