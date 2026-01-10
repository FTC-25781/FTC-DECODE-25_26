package org.firstinspires.ftc.teamcode.layeredOld.physical1;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeMotor {
    private DcMotorEx intakeMotor;
    private LynxModule hub;

    private int targetPosition = 0;
    private boolean moving = false;

    public IntakeMotor(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "imot");
        hub = hardwareMap.get(LynxModule.class, "Control Hub");
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void startIntaking() {
        intakeMotor.setPower(0.8);
    }

    public void returnToIdle() {
        intakeMotor.setPower(0);
    }

    public void startRemoving() {
        intakeMotor.setPower(-1);
    }

    public void update() {

    }
}
