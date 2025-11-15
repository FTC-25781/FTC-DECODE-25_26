package org.firstinspires.ftc.teamcode.layered.physical1;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoForSorter {
    public ServoImplEx transfer;

    double[] positions = {0.06, 0.44, 0.82}; // 0-2 Intake 3-5 Deposit | 0.19, 0.56, 0.94,  0 is purple, 1 is purple, 2 is green

    public int current_Pos = 0;

    public ServoForSorter(HardwareMap hardwareMap) {
        transfer = hardwareMap.get(ServoImplEx.class, "transfer1");
        transfer.setPwmEnable();
//        transfer.setPosition(0.16);
    }

    public void GoForwards() {
        if (current_Pos != 2) {
            current_Pos += 1;
            transfer.setPosition(positions[current_Pos]);
        }
    }

    public void goTo0() {
        current_Pos = 0;
        transfer.setPosition(positions[current_Pos]);
    }

    public void goTo1() {
        current_Pos = 1;
        transfer.setPosition(positions[current_Pos]);
    }

    public void goTo2() {
        current_Pos = 2;
        transfer.setPosition(positions[current_Pos]);
    }

    public void GoBackwards() {
        if (current_Pos != 0) {
            current_Pos -= 1;
            transfer.setPosition(positions[current_Pos]);
        }
    }

    public void update(Telemetry telemetry) {

        telemetry.addData("Current State", current_Pos);
        telemetry.addData("Servo Pos", transfer.getPosition());
    }
}
