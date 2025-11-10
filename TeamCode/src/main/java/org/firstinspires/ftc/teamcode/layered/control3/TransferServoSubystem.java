package org.firstinspires.ftc.teamcode.layered.control3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.layered.physical1.CRServoForTransfer;

public class TransferServoSubystem {

    private final CRServoForTransfer servoHardware;
    private final ElapsedTime debounceTimer = new ElapsedTime();

    public TransferServoSubystem(HardwareMap hardwareMap) {
        servoHardware = new CRServoForTransfer(hardwareMap);
    }

    public void update() {
        servoHardware.update();
    }
}
