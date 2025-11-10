package org.firstinspires.ftc.teamcode.layered.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test Mini Transfer", group = "Test")
public class TestMiniTransfer extends OpMode {
    private CRServo miniTransfer;
    private DigitalChannel digitalTouch;

    private ElapsedTime timer;
    private ElapsedTime buttonDebounce;

    private boolean waitingForDelay = false;
    private static final double DELAY_TIME = 2.0;

    @Override
    public void init() {
        miniTransfer = hardwareMap.get(CRServo.class, "transfer2");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        timer = new ElapsedTime();
        buttonDebounce = new ElapsedTime();
    }

    @Override
    public void loop() {
        boolean touchPressed = !digitalTouch.getState();

        if (gamepad1.dpad_down && buttonDebounce.seconds() > 0.2) {
            if (touchPressed) {
                miniTransfer.setPower(1);
                buttonDebounce.reset();
            } else {
                telemetry.addLine("Cannot move down - touch sensor pressed!");
            }
        }


        if (gamepad1.dpad_up && buttonDebounce.seconds() > 0.2 && !waitingForDelay) {
            timer.reset();
            waitingForDelay = true;
            buttonDebounce.reset();
        }

        if (waitingForDelay && timer.seconds() >= DELAY_TIME) {
            if (touchPressed) {
                miniTransfer.setPower(-1);
                buttonDebounce.reset();
            } else {
                telemetry.addLine("Cannot move up - touch sensor pressed!");
            }
            waitingForDelay = false;
        }

        telemetry.addData("Touch Sensor", touchPressed ? "PRESSED" : "Not Pressed");
        telemetry.addData("Waiting for delay", waitingForDelay);
        if (waitingForDelay) {
            telemetry.addData("Time remaining", "%.1f sec", DELAY_TIME - timer.seconds());
        }
        telemetry.update();
    }
}