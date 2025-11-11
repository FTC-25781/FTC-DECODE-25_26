package org.firstinspires.ftc.teamcode.layered.physical1;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CRServoForSorter {
    public CRServo transfer;

    public CRServoForSorter(HardwareMap hardwareMap) {
        transfer = hardwareMap.get(CRServo.class,"transfer1");
        transfer.setPower(0);
    }

    public void StartRotation() {
        transfer.setPower(0.2);
    }

    public void StartRotationReverse() {
        transfer.setPower(-0.2);
    }

    public void StopServo() {
        transfer.setPower(0.0);
    }


}
