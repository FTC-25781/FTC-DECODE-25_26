package org.firstinspires.ftc.teamcode.layered.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MaxSpeedMotorTest")
public class MaxSpeedMotorTest extends LinearOpMode {
    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motor from the robot configuration
        motor = hardwareMap.get(DcMotor.class, "motorName"); // replace "motorName" with your config name

        waitForStart();

        while (opModeIsActive()) {
            // Set motor to max forward speed
            motor.setPower(gamepad1.left_stick_x);

            // or use -1.0 for reverse
            // motor.setPower(-1.0);

            telemetry.addData("Motor Power", motor.getPower());
            telemetry.update();
        }
    }
}
