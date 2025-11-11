package org.firstinspires.ftc.teamcode.layered.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Encoder + CRServo", group = "Test")
public class MotEncoderPlusIntakeServo extends LinearOpMode {

    private DcMotorEx encoder;
    private CRServo servo;
    private int target = 400; // 60° at 2400 ticks/rev

    @Override
    public void runOpMode() throws InterruptedException {
        encoder = hardwareMap.get(DcMotorEx.class, "encoder");
        servo = hardwareMap.get(CRServo.class, "transfer1");

        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder.setDirection(DcMotorSimple.Direction.REVERSE);

        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        while (opModeIsActive()) {
            if (Math.abs(encoder.getCurrentPosition()) > target) {
                servo.setPower(0);
            }
            if(gamepad1.a)
            {
                encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                servo.setPower(0.2);
            }

            telemetry.addData("Encoder position", encoder.getCurrentPosition());
            telemetry.update();


        }
    }
}
