package org.firstinspires.ftc.teamcode.layeredOld.control3;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Deposit {
    private Servo miniTransfer;
    private DigitalChannel digitalTouch;

    public enum DEPOSIT_STATE {
        IDLE,
        TRANSFERRING,
        WAITING_FOR_BALL
    }

    private DEPOSIT_STATE currentState = DEPOSIT_STATE.IDLE;

    private final ElapsedTime transferTimer = new ElapsedTime();

    // Constants
    private static final double TRANSFER_SERVO_IN = 0.0;   // Servo position to receive ball
    private static final double TRANSFER_SERVO_OUT = 1.0;  // Servo position to push ball to shooter
    private static final double TRANSFER_DURATION = 0.5;   // Time to complete transfer

    public Deposit(HardwareMap hardwareMap) {
        miniTransfer = hardwareMap.get(Servo.class, "transfer2");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch"); // <-- Name must match RC config
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        miniTransfer.setPosition(TRANSFER_SERVO_IN);
        transferTimer.reset();
    }

    // --- Control Methods ---
    public void startTransfer() {
        // Only start transfer if no ball is blocking bottom sensor
        if (digitalTouch.getState()) {
            currentState = DEPOSIT_STATE.TRANSFERRING;
            transferTimer.reset();
        } else {
            currentState = DEPOSIT_STATE.WAITING_FOR_BALL;
        }
    }

    public void returnToIdle() {
        currentState = DEPOSIT_STATE.IDLE;
    }

    public void reset() {
        currentState = DEPOSIT_STATE.IDLE;
        miniTransfer.setPosition(TRANSFER_SERVO_IN);
    }

    // --- State Helpers ---
    public DEPOSIT_STATE getState() {
        return currentState;
    }

    public boolean isIdle() {
        return currentState == DEPOSIT_STATE.IDLE;
    }

    public boolean isTransferring() {
        return currentState == DEPOSIT_STATE.TRANSFERRING;
    }

    // --- Main Update Loop ---
    public void update() {
        switch (currentState) {
            case IDLE:
                if (digitalTouch.getState()) {
                    miniTransfer.setPosition(miniTransfer.getPosition() - 0.1);
                }
                break;

            case WAITING_FOR_BALL:
                if (digitalTouch.getState()) {
                    currentState = DEPOSIT_STATE.TRANSFERRING;
                    transferTimer.reset();
                }
                break;

            case TRANSFERRING:
                miniTransfer.setPosition(TRANSFER_SERVO_OUT);

                if (transferTimer.seconds() > TRANSFER_DURATION) {
                    miniTransfer.setPosition(miniTransfer.getPosition() + 0.1);
                    currentState = DEPOSIT_STATE.IDLE;
                }
                break;
        }
    }

    // --- Telemetry ---
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Deposit State", currentState);
        telemetry.addData("Servo Position", "%.2f", miniTransfer.getPosition());
        telemetry.addData("Bottom Sensor", digitalTouch.getState() ? "PRESSED" : "NOT PRESSED");

        if (currentState == DEPOSIT_STATE.TRANSFERRING) {
            telemetry.addData("Transfer Timer", "%.2fs", transferTimer.seconds());
        }
    }

    public void stop() {
        miniTransfer.setPosition(TRANSFER_SERVO_IN);
    }
}
