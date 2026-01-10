package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name="Mecanum Drive", group="TeleOp")
public class DriveTest extends OpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void init() {
        frontLeft  = hardwareMap.dcMotor.get("LF");
        frontRight = hardwareMap.dcMotor.get("LB");
        backLeft   = hardwareMap.dcMotor.get("RF");
        backRight  = hardwareMap.dcMotor.get("RB");

        // Reverse left side
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Initialized");
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;   // forward/back
        double x =  gamepad1.left_stick_x;   // strafe
        double rx = gamepad1.right_stick_x;  // turn

        double denominator = Math.max(
                Math.abs(y) + Math.abs(x) + Math.abs(rx), 1
        );

        double frontLeftPower  = (y + x + rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower  = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
}
