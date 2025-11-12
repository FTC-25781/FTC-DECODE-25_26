package org.firstinspires.ftc.teamcode.layered.control3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.layered.physical1.ServoForTransfer;

public class LiftSubsystem {

    private final ServoForTransfer servoHardware;
    private final ElapsedTime debounceTimer = new ElapsedTime();

    public LiftSubsystem(HardwareMap hardwareMap) {
        servoHardware = new ServoForTransfer(hardwareMap);
    }

    public void update() {
        servoHardware.update();
    }
}