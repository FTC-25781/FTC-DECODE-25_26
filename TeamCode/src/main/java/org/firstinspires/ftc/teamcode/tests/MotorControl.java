package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name="Motor Control")
public class MotorControl extends SelectableOpMode {
    public MotorControl() {
        super("Select a Motor Control", s -> {
            s.folder("RPM", l -> {
                l.add("Bare Motor RPM (PIDF tuned a little)", BareMotor::new);
                l.add("6000 RPM (PIDF tuned a little)", RPM6000::new);
                l.add("1620 RPM (not Tuned)", RPM1620::new);
                l.add("1150 RPM (not Tuned)", RPM1150::new);
                l.add("435 RPM (not Tuned)", RPM435::new);
                l.add("312 RPM (not Tuned)", RPM312::new);
                l.add("223 RPM (not Tuned)", RPM223::new);
                l.add("117 RPM (not Tuned)", RPM117::new);
                l.add("84 RPM (not Tuned)", RPM84::new);
                l.add("60 RPM (not Tuned)", RPM60::new);
                l.add("43 RPM (not Tuned)", RPM43::new);
                l.add("30 RPM (not Tuned)", RPM30::new);
            });
            s.folder("Position (nothing here)", l -> {
//                l.add("6000 RPM (PIDF tuned a little)", RPM6000::new);

                l.add("1150 Pos (not Tuned)", Pos1150::new);
//                l.add("435 RPM (not Tuned)", RPM435::new);
//                l.add("312 RPM (not Tuned)", RPM312::new);
//                l.add("223 RPM (not Tuned)", RPM223::new);
//                l.add("117 RPM (not Tuned)", RPM117::new);
//                l.add("84 RPM (not Tuned)", RPM84::new);
//                l.add("60 RPM (not Tuned)", RPM60::new);
//                l.add("43 RPM (not Tuned)", RPM43::new);
//                l.add("30 RPM (not Tuned)", RPM30::new);
            });
        });
    }

}

class BareMotor extends OpMode {
    DcMotorEx motor;

    double TPR = 28;

    double targetRPM = 0;
    double targetTicks = 0;

    boolean dpadr = false;
    boolean dpadl = false;
    boolean dpadu = false;
    boolean dpadd = false;
    boolean lastB = false;
    boolean lastX = false;

    boolean lastRB = false;
    boolean lastLB = false;
    boolean lastRT = false;
    boolean lastLT = false;
    double maxRPM=0;

    double kP = 0;
    double kI = 0;
    double kD = 0;
    double kF = 1;



    double minRPM=10000;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorImplEx.class, "motor");


        // IMPORTANT: Reset the encoder first
        motor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(DcMotorEx.Direction.REVERSE);

        // Then run using encoder
        motor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        motor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
        telemetry.addData("TargetPosition", motor.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            targetRPM = 0;
        }
        else if (gamepad1.right_stick_button){
            targetRPM=3500;
        }else if (gamepad1.y){
            targetRPM = 4600;
        } else if (gamepad1.dpad_right && !dpadr) {
            targetRPM += 10;
        } else if (gamepad1.dpad_left && !dpadl) {
            targetRPM -= 10;
        } else if ((gamepad1.dpad_up && !dpadu) && (!gamepad1.b || !gamepad1.x)) {
            targetRPM += 300;
        } else if ((gamepad1.dpad_down && !dpadd) && (!gamepad1.b || !gamepad1.x)) {
            targetRPM -= 300;
        }

        targetTicks = targetRPM * TPR / 60;

//idk about changing the pidf coefficients but i did so because too much fluctuation
        motor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        //original was motor.setVelocityPIDFCoefficients(10,3 ,0,0);


        // ----- kP -----
        if (gamepad1.right_bumper && !lastRB) {
            kP += 1;
        }
        if (gamepad1.left_bumper && !lastLB) {
            kP -= 1;
        }

// ----- kD -----
        if (gamepad1.right_trigger > 0.5 && !lastRT) {
            kD += 0.25;
        }
        if (gamepad1.left_trigger > 0.5 && !lastLT) {
            kD -= 0.25;
        }

// ----- kI (hold X) -----
        if (gamepad1.x && gamepad1.dpad_up && !dpadu) {
            kI += 0.1;
        }
        if (gamepad1.x && gamepad1.dpad_down && !dpadd) {
            kI -= 0.1;
        }

// ----- kF (hold B) -----
        if (gamepad1.b && !lastB) {
            kF += 0.5;
        }
        if (gamepad1.startWasPressed()) {
            kF -= 0.5;
        }


        motor.setVelocity(targetTicks);
//        if (gamepad1.y){
//            if ((motor.getVelocity() * 60 / TPR)<minRPM){
//                minRPM = motor.getVelocity() * 60 / TPR;
//            }
//            if ((motor.getVelocity() * 60 / TPR)>maxRPM){
//                maxRPM = motor.getVelocity() * 60 / TPR;
//            }
//        }

        dpadr = gamepad1.dpad_right;
        dpadl = gamepad1.dpad_left;
        dpadu = gamepad1.dpad_up;
        dpadd = gamepad1.dpad_down;
        lastB = gamepad1.b;
        lastX = gamepad1.x;

        lastRB = gamepad1.right_bumper;
        lastLB = gamepad1.left_bumper;
        lastRT = gamepad1.right_trigger > 0.5;
        lastLT = gamepad1.left_trigger > 0.5;


        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", motor.getVelocity() * 60 / TPR);
        telemetry.addData("Min RPM", minRPM);
        telemetry.addData("Max RPM", maxRPM);

        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Velocity", motor.getVelocity());


        telemetry.addData("TPR", TPR);


        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Coeff", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.update();

    }

}

class RPM6000 extends OpMode {
    DcMotorImplEx motor;

    //        double TPR = 28;
    double TPR =28;
    ElapsedTime timer = new ElapsedTime();

    double targetRPM = 0;
    double targetTicks = 0;
    boolean dpadr = false;
    boolean dpadl = false;
    boolean dpadu = false;
    boolean dpadd = false;

    double maxRPM=0;
    private Follower follower;
    public static Pose startingPose = new Pose(72,72,45);

    double minRPM=10000;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorImplEx.class, "motor");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        // IMPORTANT: Reset the encoder first
        motor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);

        // Then run using encoder
        motor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        motor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
        telemetry.addData("TargetPosition", motor.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();
        if (gamepad1.a) {
            targetRPM = 0;
        } else if (gamepad1.x) {
            targetRPM = 1;
        } else if (gamepad1.b) {
            targetRPM = 5;
        } else if (gamepad1.dpad_right && !dpadr) {
            targetRPM += 20;
        } else if (gamepad1.dpad_left && !dpadl) {
            targetRPM -= 10;
        } else if (gamepad1.dpad_up && !dpadu) {
            targetRPM += 300;
        } else if (gamepad1.dpad_down && !dpadd) {
            targetRPM -= 300;
        }



        targetTicks = targetRPM * TPR / 60;


//idk about changing the pidf coefficients but i did so because too much fluctuation
        motor.setVelocityPIDFCoefficients(9.25, 2.2, 7, 0);
        //original was motor.setVelocityPIDFCoefficients(10,3 ,0,0);

        motor.setVelocity(targetTicks);

        if (gamepad1.y){
            if ((motor.getVelocity() * 60 / TPR)<minRPM){
                minRPM = motor.getVelocity() * 60 / TPR;
            }
            if ((motor.getVelocity() * 60 / TPR)>maxRPM){
                maxRPM = motor.getVelocity() * 60 / TPR;
            }
        }

        dpadr = gamepad1.dpad_right;
        dpadl = gamepad1.dpad_left;
        dpadu = gamepad1.dpad_up;
        dpadd = gamepad1.dpad_down;

        telemetry.addData("Distance",Math.sqrt(Math.pow(follower.getPose().getX()-144,2)+(Math.pow(follower.getPose().getY()-144,2))));
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", motor.getVelocity() * 60 / TPR);
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("heading",follower.getPose().getHeading());



        telemetry.addData("Min RPM", minRPM);
        telemetry.addData("Max RPM", maxRPM);

        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Velocity", motor.getVelocity());


        telemetry.addData("TPR", TPR);


        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Coeff", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.update();

    }

}
class RPM1620 extends OpMode {
    DcMotorImplEx motor;

    double TPR = 28;

    double targetRPM = 0;
    double targetTicks = 0;

    boolean dpadr = false;
    boolean dpadl = false;
    boolean dpadu = false;
    boolean dpadd = false;
    double maxRPM=0;


    double minRPM=10000;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorImplEx.class, "motor");


        // IMPORTANT: Reset the encoder first
        motor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);

        // Then run using encoder
        motor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        motor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
        telemetry.addData("TargetPosition", motor.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            targetRPM = 0;
        } else if (gamepad1.x) {
            targetRPM = 1;
        } else if (gamepad1.b) {
            targetRPM = 5;
        } else if (gamepad1.dpad_right && !dpadr) {
            targetRPM += 10;
        } else if (gamepad1.dpad_left && !dpadl) {
            targetRPM -= 10;
        } else if (gamepad1.dpad_up && !dpadu) {
            targetRPM += 100;
        } else if (gamepad1.dpad_down && !dpadd) {
            targetRPM -= 100;
        }

        targetTicks = targetRPM * TPR / 60;

//idk about changing the pidf coefficients but i did so because too much fluctuation
        motor.setVelocityPIDFCoefficients(9.25, 2.2, 7, 0);
        //original was motor.setVelocityPIDFCoefficients(10,3 ,0,0);

        motor.setVelocity(targetTicks);
        if (gamepad1.y){
            if ((motor.getVelocity() * 60 / TPR)<minRPM){
                minRPM = motor.getVelocity() * 60 / TPR;
            }
            if ((motor.getVelocity() * 60 / TPR)>maxRPM){
                maxRPM = motor.getVelocity() * 60 / TPR;
            }
        }

        dpadr = gamepad1.dpad_right;
        dpadl = gamepad1.dpad_left;
        dpadu = gamepad1.dpad_up;
        dpadd = gamepad1.dpad_down;
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", motor.getVelocity() * 60 / TPR);
        telemetry.addData("Min RPM", minRPM);
        telemetry.addData("Max RPM", maxRPM);

        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Velocity", motor.getVelocity());


        telemetry.addData("TPR", TPR);


        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Coeff", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.update();

    }

}
class RPM1150 extends OpMode {
    DcMotorImplEx motor;

    double TPR = 28 ;

    double targetRPM = 0;
    double targetTicks = 0;

    boolean dpadr = false;
    boolean dpadl = false;
    boolean dpadu = false;
    boolean dpadd = false;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorImplEx.class, "motor");


        // IMPORTANT: Reset the encoder first
        motor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);

        // Then run using encoder
        motor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        motor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
        telemetry.addData("TargetPosition", motor.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            targetRPM = 0;
        } else if (gamepad1.x) {
            targetRPM = 1;
        } else if (gamepad1.b) {
            targetRPM = 5;
        } else if (gamepad1.dpad_right && !dpadr) {
            targetRPM += 10;
        } else if (gamepad1.dpad_left && !dpadl) {
            targetRPM -= 10;
        } else if (gamepad1.dpad_up && !dpadu) {
            targetRPM += 100;
        } else if (gamepad1.dpad_down && !dpadd) {
            targetRPM -= 100;
        }

        targetTicks = targetRPM * TPR / 60;

//idk about changing the pidf coefficients but i did so because too much fluctuation
        motor.setVelocityPIDFCoefficients(9.25, 2.2, 7, 0);
        //original was motor.setVelocityPIDFCoefficients(10,3 ,0,0);

        motor.setVelocity(targetTicks);

        dpadr = gamepad1.dpad_right;
        dpadl = gamepad1.dpad_left;
        dpadu = gamepad1.dpad_up;
        dpadd = gamepad1.dpad_down;
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", motor.getVelocity() * 60 / TPR);
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Velocity", motor.getVelocity());


        telemetry.addData("TPR", TPR);


        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Coeff", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.update();

    }

}  //Add the min and max counter to this opmode and below ones

class RPM435 extends OpMode {
    DcMotorImplEx motor;

    double TPR = 28 ;

    double targetRPM = 0;
    double targetTicks = 0;

    boolean dpadr = false;
    boolean dpadl = false;
    boolean dpadu = false;
    boolean dpadd = false;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorImplEx.class, "motor");


        // IMPORTANT: Reset the encoder first
        motor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);

        // Then run using encoder
        motor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        motor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
        telemetry.addData("TargetPosition", motor.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            targetRPM = 0;
        } else if (gamepad1.x) {
            targetRPM = 1;
        } else if (gamepad1.b) {
            targetRPM = 5;
        } else if (gamepad1.dpad_right && !dpadr) {
            targetRPM += 5;
        } else if (gamepad1.dpad_left && !dpadl) {
            targetRPM -= 5;
        } else if (gamepad1.dpad_up && !dpadu) {
            targetRPM += 50;
        } else if (gamepad1.dpad_down && !dpadd) {
            targetRPM -= 50;
        }

        targetTicks = targetRPM * TPR / 60;

//idk about changing the pidf coefficients but i did so because too much fluctuation
        motor.setVelocityPIDFCoefficients(9.25, 2.2, 7, 0);
        //original was motor.setVelocityPIDFCoefficients(10,3 ,0,0);

        motor.setVelocity(targetTicks);

        dpadr = gamepad1.dpad_right;
        dpadl = gamepad1.dpad_left;
        dpadu = gamepad1.dpad_up;
        dpadd = gamepad1.dpad_down;
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", motor.getVelocity() * 60 / TPR);
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Velocity", motor.getVelocity());


        telemetry.addData("TPR", TPR);


        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Coeff", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.update();

    }

}

class RPM312 extends OpMode {
    DcMotorImplEx motor;

    double TPR = 28 ;

    double targetRPM = 0;
    double targetTicks = 0;

    boolean dpadr = false;
    boolean dpadl = false;
    boolean dpadu = false;
    boolean dpadd = false;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorImplEx.class, "motor");


        // IMPORTANT: Reset the encoder first
        motor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);

        // Then run using encoder
        motor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        motor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
        telemetry.addData("TargetPosition", motor.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            targetRPM = 0;
        } else if (gamepad1.x) {
            targetRPM = 2;
        } else if (gamepad1.b) {
            targetRPM = 5;
        } else if (gamepad1.dpad_right && !dpadr) {
            targetRPM += 5;
        } else if (gamepad1.dpad_left && !dpadl) {
            targetRPM -= 5;
        } else if (gamepad1.dpad_up && !dpadu) {
            targetRPM += 50;
        } else if (gamepad1.dpad_down && !dpadd) {
            targetRPM -= 50;
        }

        targetTicks = targetRPM * TPR / 60;

//idk about changing the pidf coefficients but i did so because too much fluctuation
        motor.setVelocityPIDFCoefficients(9.25, 2.2, 7, 0);
        //original was motor.setVelocityPIDFCoefficients(10,3 ,0,0);

        motor.setVelocity(targetTicks);

        dpadr = gamepad1.dpad_right;
        dpadl = gamepad1.dpad_left;
        dpadu = gamepad1.dpad_up;
        dpadd = gamepad1.dpad_down;
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", motor.getVelocity() * 60 / TPR);
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Velocity", motor.getVelocity());


        telemetry.addData("TPR", TPR);


        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Coeff", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.update();

    }

}
class RPM223 extends OpMode {
    DcMotorImplEx motor;

    double TPR = 28;

    double targetRPM = 0;
    double targetTicks = 0;

    boolean dpadr = false;
    boolean dpadl = false;
    boolean dpadu = false;
    boolean dpadd = false;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorImplEx.class, "motor");


        // IMPORTANT: Reset the encoder first
        motor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);

        // Then run using encoder
        motor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        motor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
        telemetry.addData("TargetPosition", motor.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            targetRPM = 0;
        } else if (gamepad1.x) {
            targetRPM = 3;
        } else if (gamepad1.b) {
            targetRPM = 5;
        } else if (gamepad1.dpad_right && !dpadr) {
            targetRPM += 5;
        } else if (gamepad1.dpad_left && !dpadl) {
            targetRPM -= 5;
        } else if (gamepad1.dpad_up && !dpadu) {
            targetRPM += 40;
        } else if (gamepad1.dpad_down && !dpadd) {
            targetRPM -= 40;
        }

        targetTicks = targetRPM * TPR / 60;

//idk about changing the pidf coefficients but i did so because too much fluctuation
        motor.setVelocityPIDFCoefficients(9.25, 2.2, 7, 0);
        //original was motor.setVelocityPIDFCoefficients(10,3 ,0,0);

        motor.setVelocity(targetTicks);

        dpadr = gamepad1.dpad_right;
        dpadl = gamepad1.dpad_left;
        dpadu = gamepad1.dpad_up;
        dpadd = gamepad1.dpad_down;
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", motor.getVelocity() * 60 / TPR);
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Velocity", motor.getVelocity());


        telemetry.addData("TPR", TPR);


        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Coeff", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.update();

    }

}
class RPM117 extends OpMode {
    DcMotorImplEx motor;

    double TPR = 28 ;

    double targetRPM = 0;
    double targetTicks = 0;

    boolean dpadr = false;
    boolean dpadl = false;
    boolean dpadu = false;
    boolean dpadd = false;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorImplEx.class, "motor");


        // IMPORTANT: Reset the encoder first
        motor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);

        // Then run using encoder
        motor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        motor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
        telemetry.addData("TargetPosition", motor.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            targetRPM = 0;
        } else if (gamepad1.x) {
            targetRPM = 2;
        } else if (gamepad1.b) {
            targetRPM = 5;
        } else if (gamepad1.dpad_right && !dpadr) {
            targetRPM += 15;
        } else if (gamepad1.dpad_left && !dpadl) {
            targetRPM -= 5;
        } else if (gamepad1.dpad_up && !dpadu) {
            targetRPM += 25;
        } else if (gamepad1.dpad_down && !dpadd) {
            targetRPM -= 25;
        }

        targetTicks = targetRPM * TPR / 60;

//idk about changing the pidf coefficients but i did so because too much fluctuation
        motor.setVelocityPIDFCoefficients(9.25, 2.2, 7, 0);
        //original was motor.setVelocityPIDFCoefficients(10,3 ,0,0);

        motor.setVelocity(targetTicks);

        dpadr = gamepad1.dpad_right;
        dpadl = gamepad1.dpad_left;
        dpadu = gamepad1.dpad_up;
        dpadd = gamepad1.dpad_down;
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", motor.getVelocity() * 60 / TPR);
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Velocity", motor.getVelocity());


        telemetry.addData("TPR", TPR);


        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Coeff", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.update();

    }

}
class RPM84 extends OpMode {
    DcMotorImplEx motor;

    double TPR = 28 ;

    double targetRPM = 0;
    double targetTicks = 0;

    boolean dpadr = false;
    boolean dpadl = false;
    boolean dpadu = false;
    boolean dpadd = false;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorImplEx.class, "motor");


        // IMPORTANT: Reset the encoder first
        motor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);

        // Then run using encoder
        motor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        motor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
        telemetry.addData("TargetPosition", motor.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            targetRPM = 0;
        } else if (gamepad1.x) {
            targetRPM = 1;
        } else if (gamepad1.b) {
            targetRPM = 5;
        } else if (gamepad1.dpad_right && !dpadr) {
            targetRPM += 2;
        } else if (gamepad1.dpad_left && !dpadl) {
            targetRPM -= 2;
        } else if (gamepad1.dpad_up && !dpadu) {
            targetRPM += 10;
        } else if (gamepad1.dpad_down && !dpadd) {
            targetRPM -= 10;
        }

        targetTicks = targetRPM * TPR / 60;

//idk about changing the pidf coefficients but i did so because too much fluctuation
        motor.setVelocityPIDFCoefficients(9.25, 2.2, 7, 0);
        //original was motor.setVelocityPIDFCoefficients(10,3 ,0,0);

        motor.setVelocity(targetTicks);

        dpadr = gamepad1.dpad_right;
        dpadl = gamepad1.dpad_left;
        dpadu = gamepad1.dpad_up;
        dpadd = gamepad1.dpad_down;
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", motor.getVelocity() * 60 / TPR);
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Velocity", motor.getVelocity());


        telemetry.addData("TPR", TPR);


        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Coeff", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.update();

    }

}
class RPM60 extends OpMode {
    DcMotorImplEx motor;

    double TPR = 28 ;

    double targetRPM = 0;
    double targetTicks = 0;

    boolean dpadr = false;
    boolean dpadl = false;
    boolean dpadu = false;
    boolean dpadd = false;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorImplEx.class, "motor");


        // IMPORTANT: Reset the encoder first
        motor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);

        // Then run using encoder
        motor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        motor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
        telemetry.addData("TargetPosition", motor.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            targetRPM = 0;
        } else if (gamepad1.x) {
            targetRPM = 1;
        } else if (gamepad1.b) {
            targetRPM = 5;
        } else if (gamepad1.dpad_right && !dpadr) {
            targetRPM += 1;
        } else if (gamepad1.dpad_left && !dpadl) {
            targetRPM -= 1;
        } else if (gamepad1.dpad_up && !dpadu) {
            targetRPM += 5;
        } else if (gamepad1.dpad_down && !dpadd) {
            targetRPM -= 5;
        }

        targetTicks = targetRPM * TPR / 60;

//idk about changing the pidf coefficients but i did so because too much fluctuation
        motor.setVelocityPIDFCoefficients(9.25, 2.2, 7, 0);
        //original was motor.setVelocityPIDFCoefficients(10,3 ,0,0);

        motor.setVelocity(targetTicks);

        dpadr = gamepad1.dpad_right;
        dpadl = gamepad1.dpad_left;
        dpadu = gamepad1.dpad_up;
        dpadd = gamepad1.dpad_down;
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", motor.getVelocity() * 60 / TPR);
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Velocity", motor.getVelocity());


        telemetry.addData("TPR", TPR);


        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Coeff", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.update();

    }

}
class RPM43 extends OpMode {
    DcMotorImplEx motor;

    double TPR = 28 ;

    double targetRPM = 0;
    double targetTicks = 0;

    boolean dpadr = false;
    boolean dpadl = false;
    boolean dpadu = false;
    boolean dpadd = false;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorImplEx.class, "motor");


        // IMPORTANT: Reset the encoder first
        motor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);

        // Then run using encoder
        motor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        motor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
        telemetry.addData("TargetPosition", motor.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            targetRPM = 0;
        } else if (gamepad1.x) {
            targetRPM = 1;
        } else if (gamepad1.b) {
            targetRPM = 5;
        } else if (gamepad1.dpad_right && !dpadr) {
            targetRPM +=1;
        } else if (gamepad1.dpad_left && !dpadl) {
            targetRPM -= 1;
        } else if (gamepad1.dpad_up && !dpadu) {
            targetRPM += 4;
        } else if (gamepad1.dpad_down && !dpadd) {
            targetRPM -= 4;
        }

        targetTicks = targetRPM * TPR / 60;

//idk about changing the pidf coefficients but i did so because too much fluctuation
        motor.setVelocityPIDFCoefficients(9.25, 2.2, 7, 0);
        //original was motor.setVelocityPIDFCoefficients(10,3 ,0,0);

        motor.setVelocity(targetTicks);

        dpadr = gamepad1.dpad_right;
        dpadl = gamepad1.dpad_left;
        dpadu = gamepad1.dpad_up;
        dpadd = gamepad1.dpad_down;
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", motor.getVelocity() * 60 / TPR);
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Velocity", motor.getVelocity());


        telemetry.addData("TPR", TPR);


        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Coeff", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.update();

    }

}
class RPM30 extends OpMode {
    DcMotorImplEx motor;

    double TPR = 28 ;

    double targetRPM = 0;
    double targetTicks = 0;

    boolean dpadr = false;
    boolean dpadl = false;
    boolean dpadu = false;
    boolean dpadd = false;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorImplEx.class, "motor");


        // IMPORTANT: Reset the encoder first
        motor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);

        // Then run using encoder
        motor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        motor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
        telemetry.addData("TargetPosition", motor.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            targetRPM = 0;
        } else if (gamepad1.x) {
            targetRPM = 1;
        } else if (gamepad1.b) {
            targetRPM = 5;
        } else if (gamepad1.dpad_right && !dpadr) {
            targetRPM += 1;
        } else if (gamepad1.dpad_left && !dpadl) {
            targetRPM -= 1;
        } else if (gamepad1.dpad_up && !dpadu) {
            targetRPM += 3;
        } else if (gamepad1.dpad_down && !dpadd) {
            targetRPM -= 3;
        }

        targetTicks = targetRPM * TPR / 60;

//idk about changing the pidf coefficients but i did so because too much fluctuation
        motor.setVelocityPIDFCoefficients(9.25, 2.2, 7, 0);
        //original was motor.setVelocityPIDFCoefficients(10,3 ,0,0);

        motor.setVelocity(targetTicks);

        dpadr = gamepad1.dpad_right;
        dpadl = gamepad1.dpad_left;
        dpadu = gamepad1.dpad_up;
        dpadd = gamepad1.dpad_down;
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", motor.getVelocity() * 60 / TPR);
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Velocity", motor.getVelocity());


        telemetry.addData("TPR", TPR);


        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Coeff", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.update();

    }

}

class Pos1150 extends OpMode{
    DcMotorImplEx motor;
    double kP = 0.001;
    double kI = 0;
    double kD = 0;
    double kF = 0;

    boolean lastRB = false;
    boolean lastLB = false;
    boolean lastRT = false;
    boolean lastLT = false;
    PIDFController turretPID = new PIDFController(new com.pedropathing.control.PIDFCoefficients(kP, kI, kD, kF));

    int targetPos =0;

    @Override
    public void init(){
        motor = hardwareMap.get(DcMotorImplEx.class, "tmot");


        // IMPORTANT: Reset the encoder first
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motor.setDirection(DcMotorEx.Direction.REVERSE);

        // Then run using encoder
//        motor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);


        turretPID.setTargetPosition(0);
//        motor.setPower(0.5); // REQUIRED
    }

    @Override
    public void loop(){

        if(gamepad1.aWasPressed()){
            targetPos+=50;
        }
        else if(gamepad1.yWasPressed()){
            targetPos-=50;
        }

        if (gamepad1.right_bumper && !lastRB) {
            kP += 0.001;
        }
        if (gamepad1.left_bumper && !lastLB) {
            kP -= 0.001;
        }

// ----- kD -----
        if (gamepad1.right_trigger > 0.5 && !lastRT) {
            kD += 0.0001;
        }
        if (gamepad1.left_trigger > 0.5 && !lastLT) {
            kD -= 0.0001;
        }

        lastRB = gamepad1.right_bumper ;
        lastLB = gamepad1.left_bumper;
        lastRT = gamepad1.right_trigger > 0.5;
        lastLT = gamepad1.left_trigger > 0.5;
        turretPID.setCoefficients(new com.pedropathing.control.PIDFCoefficients(kP, kI, kD, kF));
        turretPID.setTargetPosition(targetPos);
//        motor.setTargetPosition(targetPos);

        turretPID.updatePosition(motor.getCurrentPosition());
        motor.setPower(turretPID.run());




        telemetry.addData("EncoderPos", motor.getCurrentPosition());
        telemetry.addData("Pidf", turretPID.getCoefficients());
        telemetry.addData("target", targetPos);
        telemetry.update();
    }
}