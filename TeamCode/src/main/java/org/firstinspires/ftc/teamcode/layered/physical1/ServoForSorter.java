package org.firstinspires.ftc.teamcode.layered.physical1;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoForSorter {
    public Servo transfer;

    double[] positions = {0.73, 0.01, 0.34}; // 0-2 Intake 3-5 Deposit | 0.19, 0.56, 0.94,

    int current_Pos = 0;

    public ServoForSorter(HardwareMap hardwareMap) {
        transfer = hardwareMap.get(Servo.class, "transfer1");
//        transfer.setPosition(0.16);
    }

    public void GoForwards() {
        if (current_Pos != 2) {
            current_Pos += 1;
            transfer.setPosition(positions[current_Pos]);
        }
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
