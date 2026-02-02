package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(name = "Jesperstest")
public class Jesperstest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double lastTime = 0;
    private double deltaTime = 0;
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotorEx kanonl;
    private DcMotorEx kanonr;
    float speed = 0;
    public double integralsum = 0;
    double kp = 0;
    double ki = 0;
    double kd = 0;
    double preverror = 0;
    private CRServo Intake;
    private CRServo Shoot;
    double driveSlow = 0.5;
    double driveSpeed;
    double fx = 400;
    double fy = 400;
    double cx = 640 / 2.0; // = 320
    double cy = 480 / 2.0; // = 240
    double rx;
    int turning = 0;
    int target = -3300;
    int tolerance;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Intake = hardwareMap.get(CRServo.class, "Intake");
        //rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        //leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        //leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        //rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        //Shoot = hardwareMap.get(CRServo.class, "Shoot");
        kanonl = hardwareMap.get(DcMotorEx.class, "Flywheell");
        //kanonl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); //BELANGRIJK
        kanonl.setDirection(DcMotorSimple.Direction.REVERSE);
        kanonr = hardwareMap.get(DcMotorEx.class, "Flywheelr");
        //kanonr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); //BELANGRIJK
        //rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        //leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        //leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        kanonr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                deltaTime = runtime.seconds() - lastTime;
                lastTime = runtime.seconds();
                test();
                intake();
                revolver();
                telemetry();

            }
        }


    }

    private void test() {
        /*if (gamepad1.left_trigger > 0.1) {
            Shoot.setPower(-1);
        } else if (gamepad1.left_bumper) {
            Shoot.setPower(1);
        } else {
            Shoot.setPower(0);
        }*/
        if (gamepad1.dpad_up) {
            speed += 1;
        } else if (gamepad1.dpad_down) {
            speed -= 1;
        } else if (gamepad1.dpad_left) {
            speed = 2900 * 28 / 60; //snelheid van kleine driehoek
        } else if (gamepad1.dpad_right) {
            speed = 2500 * 28 / 60; //snelheid van grote driehoek
        }

        if (gamepad1.cross) {
            speed = 0;
        }
        if (gamepad1.circle) {
            speed = 1000;
        } else {
            kanonl.setVelocity(speed);

        }
    }
    private void intake(){
        if (gamepad1.left_bumper){
            Intake.setPower(0.0);
        } else if (gamepad1.left_trigger > 0.4){
            Intake.setPower(1.0);
        }
    }
    private void revolver(){

        tolerance = 50;
        if (gamepad1.right_trigger > 0.4) {
            target += 10;
        } else if (gamepad1.right_bumper){
            target -= 10;
        }
        if (gamepad1.triangle){
            turning = 1;
        }
        if (turning == 1){
            if (kanonr.getCurrentPosition() > target) {
                kanonl.setVelocity(200);
            } else {
                kanonl.setVelocity(-200);
            }
        }
        if (java.lang.Math.abs(kanonr.getCurrentPosition() -  target) < tolerance){
            kanonl.setPower(0);
            turning = 0;
        }
    }
    private void telemetry(){
        telemetry.addData("speed: ",speed);
        telemetry.addData("speed encoder", kanonr.getVelocity());
        telemetry.addData("place", kanonr.getCurrentPosition());
        telemetry.addData("target", target);
        telemetry.update();
    }
}