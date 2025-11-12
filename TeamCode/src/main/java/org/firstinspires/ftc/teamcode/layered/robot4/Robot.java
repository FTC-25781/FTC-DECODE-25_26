package org.firstinspires.ftc.teamcode.layered.robot4;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.layered.control3.SorterServoSubsystem;
import org.firstinspires.ftc.teamcode.layered.control3.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.layered.physical1.ServoForTransfer;
import org.firstinspires.ftc.teamcode.layered.physical1.IntakeMotor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class Robot extends OpMode {
    private IntakeMotor mot;
    private SorterServoSubsystem servo;
    private ServoForTransfer servo_t;
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        mot = new IntakeMotor(hardwareMap);
        servo = new SorterServoSubsystem(hardwareMap);
        servo_t = new ServoForTransfer(hardwareMap);

        //shooter = new PIDShooter();

        telemetry.addData("Status", "We go to worlds????");
        telemetry.addLine();
        telemetry.addLine("Shooter Controls:");
        telemetry.addLine("DPAD LEFT - Toggle Shooter On/Off");
        telemetry.addLine("DPAD RIGHT - Toggle Alliance (Red/Blue)");
        telemetry.addLine("DPAD DOWN (Hold) - Auto-aim at goal");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Read joystick inputs
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );

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
