package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name = "CR Servo Encoder Move", group = "tests")
public class ServoTest extends LinearOpMode {

    private CRServo sorterServo;   // Continuous rotation servo
    private DcMotorEx encoder;     // Encoder motor (for tracking movement)
    public Rev2mDistanceSensor laserSensor;

    private boolean aWasPressed = false;
    private int targetPosition = 0;
    private boolean moving = false;

    @Override
    public void runOpMode() {
        sorterServo = hardwareMap.get(CRServo.class, "transfer1");
        encoder = hardwareMap.get(DcMotorEx.class, "encoder");
        laserSensor = hardwareMap.get(Rev2mDistanceSensor.class, "laser");

        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("CR Servo Encoder Test Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            int currentPos = encoder.getCurrentPosition();

            // detect rising edge of A press
            if (gamepad1.a && !aWasPressed && !moving) {
                targetPosition = currentPos + 2400;
                sorterServo.setPower(0.2);  // run at half speed forward
                moving = true;
                aWasPressed = true;
            } else if (!gamepad1.a) {
                aWasPressed = false;
            }

            // stop when encoder reaches target
            if ((moving && currentPos >= targetPosition) || (laserSensor.getDistance(DistanceUnit.MM) > 100)) {
                sorterServo.setPower(0);
                moving = false;
            }

            telemetry.addData("Encoder Pos", currentPos);
            telemetry.addData("Target Pos", targetPosition);
            telemetry.addData("Moving", moving);
            telemetry.addData("Laser Sensor", laserSensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
