package org.firstinspires.ftc.teamcode.layered.physical1;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Limit Switch Intialization
public class LimitSwitch {
    private DigitalChannel bottomSwitch;
    private DigitalChannel topSwitch;

    public LimitSwitch(HardwareMap hardwareMap) {
        bottomSwitch = hardwareMap.get(DigitalChannel.class, "bottomSwitch");
        topSwitch = hardwareMap.get(DigitalChannel.class, "topSwitch");

        bottomSwitch.setMode(DigitalChannel.Mode.INPUT);
        topSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isBottomPressed() {
        return !bottomSwitch.getState();
    }

    public boolean isTopPressed() {
        return !topSwitch.getState();
    }
}
