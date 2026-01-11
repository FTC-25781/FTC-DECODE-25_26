package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Turret Test")
public class turretRuntoPosition extends LinearOpMode {

    public DcMotorEx turretMotor;
    public static int POSITION = 200;
    public double P = 5;

    private boolean aPressed = false;
    private boolean bPressed = false;


    @Override
    public void runOpMode(){
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(P, 0, 0, 0));
        turretMotor.setPower(1);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while(opModeIsActive()){
            boolean updatePID = false;

            if(gamepad1.dpadUpWasPressed()){
                P+=0.1;
                updatePID = true;
            }
            if(gamepad1.dpadDownWasPressed()){
                P-=0.1;
                updatePID = true;
            }
            if(gamepad1.dpadRightWasPressed()){
                POSITION+=10;
            }
            if(gamepad1.dpadLeftWasPressed()){
                POSITION-=10;
            }
            if(updatePID){
                turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                        new PIDFCoefficients(P, 0, 0, 0));
            }
            turretMotor.setTargetPosition(POSITION);


            telemetry.addData("Proportional", P);
            telemetry.addData("Position", turretMotor.getTargetPosition());
            telemetry.addData("Current Position", turretMotor.getCurrentPosition());
            telemetry.addData("Error", POSITION - turretMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
