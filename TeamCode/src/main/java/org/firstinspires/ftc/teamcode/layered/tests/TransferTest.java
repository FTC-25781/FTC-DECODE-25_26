package org.firstinspires.ftc.teamcode.layered.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layered.physical1.CRServoForTransfer;

@TeleOp(name="Transfer Subsystem Test")
public class TransferTest extends LinearOpMode {

    @Override
    public void runOpMode(){

        waitForStart();
        CRServoForTransfer servo = new CRServoForTransfer(hardwareMap);

        while(opModeIsActive()){
            if(gamepad1.a)  servo.moveUp();
            if(gamepad1.b)  servo.moveDown();
            servo.update();
        }
    }
}
