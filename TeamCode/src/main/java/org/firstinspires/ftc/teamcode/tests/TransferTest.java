package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.layered.physical1.ServoForTransfer;

@Disabled
@TeleOp(name="Transfer Subsystem Test", group = "tests")
public class TransferTest extends LinearOpMode {

    @Override
    public void runOpMode(){

        waitForStart();
        ServoForTransfer servo = new ServoForTransfer(hardwareMap);

        while(opModeIsActive()){
            if(gamepad1.a)  servo.moveUp();
            if(gamepad1.b)  servo.moveDown();
            servo.update();
        }
    }
}
