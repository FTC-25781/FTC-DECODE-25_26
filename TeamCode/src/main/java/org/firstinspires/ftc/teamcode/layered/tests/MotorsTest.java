package org.firstinspires.ftc.teamcode.layered.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@Disabled
@TeleOp(name = "TestMotors", group = "tests")
public class MotorsTest extends OpMode {
//    private DcMotorEx Dmot;
    private DcMotorEx Imot;

    @Override
    public void init() {
        Imot = hardwareMap.get(DcMotorEx.class, "imot");
//        Dmot = hardwareMap.get(DcMotorEx.class, "dmot");
//        Dmot.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
//        Dmot.setPower(gamepad1.left_stick_y);
        Imot.setPower(gamepad1.right_stick_y);
//        telemetry.addData("Speed Deposit", Dmot.getPower());
        telemetry.addData("Speed Intake", Imot.getPower());
        telemetry.update();
    }
}
