package org.firstinspires.ftc.teamcode.layered.logical2;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.layered.control3.pedroPathing.Constants;

//@TeleOp(name="shooter")
public class ShooterV2 {
    private DcMotorEx shooter_motor;
    public double variation=0;

    public double power = 0;
//    public Pose startingPose = new Pose(72, 72, Math.toRadians(45)); // TODO: Integrate the pose from auto
//    private Follower follower;

    public ShooterV2(HardwareMap hardwareMap) {
        shooter_motor = hardwareMap.get(DcMotorEx.class, "dmot");
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startingPose);
    }

    //    public double calculateTargetPower(double targetRPM1) { //old
//        double x = targetRPM1;
//
//        final double C4 = 8.89811E-9;
//        final double C3 = -0.00000379115;
//        final double C2 = 0.000566876;
//        final double C1 = -0.0316266;
//        final double C0 = 1.24078;
//
//        double power = (C4 * Math.pow(x, 4)) +
//                (C3 * Math.pow(x, 3)) +
//                (C2 * Math.pow(x, 2)) +
//                (C1 * x) +
//                C0;
//
//        return Range.clip(power, 0.0, 1.0);
//    }
    public double calculateTargetPower(double targetRPM1) {
        double x = targetRPM1;

//        final double C4 = 7.13229E-9;
//        final double C3 = -(0.00000244252);
//        final double C2 = 0.000297598;
//        final double C1 = -0.0129613;
//        final double C0 = 0.819973;
//
//        double power =(C4 * Math.pow(x, 4)) +
//                (C3 * Math.pow(x, 3)) +
//                        (C2 * Math.pow(x, 2)) +
//                        (C1 * x) +
//                        C0;


//        final double C4 = 7.13229E-9;
        final double C3 = -(1.3473E-7);
        final double C2 = 0.0000454968;
        final double C1 = -0.00233777;
        final double C0 = 0.682012;

        double power =
                (C3 * Math.pow(x, 3)) +
                (C2 * Math.pow(x, 2)) +
                (C1 * x) +
                C0;

        return Range.clip(power+variation, 0.0, 1.0); // variation + 0.02
    }

//    public double targetRPM() {
        public double targetRPM(Follower follower) {
           if (follower.getPose().getY()<50){
               return Math.sqrt(Math.pow(Math.abs(follower.getPose().getX()) - 144, 2) + Math.pow(Math.abs(follower.getPose().getY()) - 144, 2));

           }else{
               return Math.sqrt(Math.pow(Math.abs(follower.getPose().getX()) - 132, 2) + Math.pow(Math.abs(follower.getPose().getY()) - 132, 2));
           }
        }

        public void reverseDepositMotor() {
            shooter_motor.setPower(-0.55);
        }

        public void shoot(double pow){
            shooter_motor.setPower(pow);

        }

        public void update(Telemetry telemetry) {
            telemetry.addData("Deposit Power", shooter_motor.getPower());
            telemetry.addData("Variation", variation);
//            telemetry.addData("Distance", targetRPM());
//            telemetry.update();

//        telemetry.update();
        }
    }