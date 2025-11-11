package org.firstinspires.ftc.teamcode.layered.robot4;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.layered.control3.SorterServoSubsystem;
import org.firstinspires.ftc.teamcode.layered.logical2.PIDShooter;
import org.firstinspires.ftc.teamcode.layered.physical1.CRServoForTransfer;
import org.firstinspires.ftc.teamcode.layered.physical1.IntakeMotor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MAIN", group = "MAIN")
public class Robot extends OpMode {
    private IntakeMotor mot;
    private SorterServoSubsystem servo;
    private CRServoForTransfer servo_t;
    private PIDShooter shooter;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;
    private boolean xWasPressed = false;
    private boolean yWasPressed = false;
    private boolean dpadDownPressed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        frontRight = hardwareMap.get(DcMotor.class, "rf");
        backLeft = hardwareMap.get(DcMotor.class, "lr");
        backRight = hardwareMap.get(DcMotor.class, "rr");

        // Reverse directions as needed for your robot
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        mot = new IntakeMotor(hardwareMap);
        servo = new SorterServoSubsystem(hardwareMap);
        servo_t = new CRServoForTransfer(hardwareMap);

        //shooter = new PIDShooter();

        telemetry.addData("Status", "67 SIGMA");
        telemetry.addLine();
        telemetry.addLine("Shooter Controls:");
        telemetry.addLine("DPAD LEFT - Toggle Shooter On/Off");
        telemetry.addLine("DPAD RIGHT - Toggle Alliance (Red/Blue)");
        telemetry.addLine("DPAD DOWN (Hold) - Auto-aim at goal");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Read joystick inputs
        double y = -gamepad1.left_stick_y; // Forward/back
        double x = gamepad1.left_stick_x * 1.1; // Strafe (tweak factor)
        double rx = gamepad1.right_stick_x; // Rotation

        // Calculate motor powers for mecanum
        double frontLeftPower = y + x + rx;
        double backLeftPower = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower = y + x - rx;

        // Normalize powers so none exceed 1.0
        double max = Math.max(1.0, Math.abs(frontLeftPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backRightPower));

        frontLeft.setPower(frontLeftPower / max);
        backLeft.setPower(backLeftPower / max);
        frontRight.setPower(frontRightPower / max);
        backRight.setPower(backRightPower / max);

        if (gamepad1.a) {
            mot.startIntaking();
        }

        if (gamepad1.b) {
            mot.returnToIdle();
        }

        if (gamepad1.x) {
            mot.startRemoving();
        }

        if (gamepad1.y) {
            servo.start();
        }

        if (gamepad1.dpad_left) {
            servo.startReverse();
        }

        if(gamepad1.dpad_up) {
            servo_t.moveUp();
        }
        if(gamepad1.dpad_down) {
            servo_t.moveDown();
        }

        servo_t.update();
        mot.update();
        servo.update();
    }
}
