package org.firstinspires.ftc.teamcode.layeredFinal.control;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DuckTrans {

    private Servo servo1;

    private final ElapsedTime stateTimer = new ElapsedTime();

    public enum State {
        IDLE,
        WORKING,
        DONE
    }

    public State currentState = State.IDLE;

    // Constructor name must match class name
    public DuckTrans(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(Servo.class, "Servo1");
    }

    public void startKickSeq() {

        // Use State instead of Transfer.State
        if (currentState == State.IDLE || currentState == State.DONE) {

            currentState = State.WORKING;

            // Move servo to 30 degrees
            servo1.setPosition(0.17);

            stateTimer.reset();
        }
    }

    public void update() {

        if (currentState == State.WORKING) {

            // Wait 0.5 seconds then finish
            if (stateTimer.seconds() > 0.5) {
                currentState = State.DONE;
            }
        }
    }
}