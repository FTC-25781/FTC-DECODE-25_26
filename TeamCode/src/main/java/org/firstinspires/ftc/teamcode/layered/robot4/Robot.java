package org.firstinspires.ftc.teamcode.layered.robot4;

import static org.firstinspires.ftc.teamcode.layered.control3.pedroPathing.Tuning.follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.layered.control3.Deposit;
import org.firstinspires.ftc.teamcode.layered.control3.Intake;
import org.firstinspires.ftc.teamcode.layered.logical2.PIDShooter;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MAIN", group = "MAIN")
public class Robot extends OpMode {
    private Intake intake;
    private Deposit deposit;
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

        deposit = new Deposit(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new PIDShooter();
        shooter.init();
        deposit.reset();

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
        double y = gamepad1.left_stick_y; // Forward/back
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

        if (gamepad1.a && !aWasPressed) {
            if (intake.getState() == Intake.INTAKE_STATE.INTAKING) {
                intake.stopIntaking();
            } else {
                intake.startIntaking();
            }
        }
        aWasPressed = gamepad1.a;

        if (gamepad1.b && !bWasPressed) {
            intake.returnToIdle();
        }
        bWasPressed = gamepad1.b;

        if (gamepad1.x && !xWasPressed) {
            if (intake.getState() == Intake.INTAKE_STATE.REVERSING) {
                intake.returnToIdle();
            } else {
                intake.reverse();
            }
        }
        xWasPressed = gamepad1.x;

        if (gamepad1.y && !yWasPressed) {
            intake.startRotation();
        }
        yWasPressed = gamepad1.y;

        if (gamepad1.dpad_up) {
            intake.reset();
        }

        if (gamepad1.right_trigger > 0.5) {
            intake.startIntaking();
        } else if (intake.getState() == Intake.INTAKE_STATE.INTAKING && gamepad1.right_trigger < 0.1) {
            intake.stopIntaking();
        }

        if (gamepad1.left_trigger > 0.5) {
            intake.reverse();
        } else if (intake.getState() == Intake.INTAKE_STATE.REVERSING && gamepad1.left_trigger < 0.1) {
            intake.returnToIdle();
        }

        intake.update();

        if (gamepad1.dpad_up) {
            deposit.startTransfer();
        }

        if (gamepad1.dpad_down) {
            deposit.returnToIdle();
        }

//        // DPAD LEFT: Toggle shooter on/off
//        if (gamepad1.dpad_left && !dpadLeftPressed) {
//            shooter.toggleEnabled();
//            if (shooter.isEnabled()) {
//                gamepad1.rumble(100); // Short rumble for feedback
//            }
//        }
//        dpadLeftPressed = gamepad1.dpad_left;
//
//        // DPAD RIGHT: Toggle alliance (Red/Blue)
//        if (gamepad1.dpad_right && !dpadRightPressed) {
//            shooter.toggleAlliance();
//            gamepad1.rumble(200); // Medium rumble for feedback
//        }
//        dpadRightPressed = gamepad1.dpad_right;
//
//        // DPAD DOWN (Hold): Auto-aim at goal
//        if (gamepad1.dpad_down) {
//            shooter.autoAimToGoal();
//
//            // Rumble when aimed and ready to fire
//            if (shooter.isAimedAtGoal() && shooter.isAtTargetSpeed(100)) {
//                gamepad1.rumble(50); // Continuous light rumble while ready
//            }
//
//            // Automatically enable shooter when auto-aiming
//            if (!shooter.isEnabled()) {
//                shooter.enable();
//            }
//        } else {
//            shooter.stopAutoAim();
//        }
//
//        // Update shooter (handles PID, odometry, ballistics)
//        shooter.update(telemetry);
        telemetry.update();
    }
}
