package org.firstinspires.ftc.teamcode.layered.control3;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Deposit {
    private DcMotorEx depositMotor;

    public enum DEPOSIT_STATE {
        IDLE,
        DEPOSITING,
        READY,
        ANGLING,
        PULLING_BALL
    }

    private DEPOSIT_STATE currentState = DEPOSIT_STATE.IDLE;
    private DEPOSIT_STATE lastState = DEPOSIT_STATE.IDLE;


}
