package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Revolverspeed")
public class Revolverspeed extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx Revolver;
    float speed = 0;
    public double integralsum = 0;
    double kp = 0.04;
    double ki = 0;
    double kd = 0;
    double preverror = 0;
    double driveSlow = 0.5;
    double driveSpeed;
    double fx = 400;
    double fy = 400;
    double cx = 640 / 2.0; // = 320
    double cy = 480 / 2.0; // = 240
    boolean revoval = true;
    double rx;
    int target;
    int RevolverTpos = 0;
    int Revolverslot = 0;
    boolean crosspressed;
    boolean circlepressed;
    double Revolverpower = 0;
    double rl = 0;
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        Revolver = hardwareMap.get(DcMotorEx.class, "Revolver");
//        Revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                revolver();
                Telemetry();
            }
        }
    }

        public void revolver() {
            if (gamepad1.cross) {
                Revolverpower += 0.0001;
                Revolver.setPower(Revolverpower);
            }

            if (gamepad1.square) {
                Revolverpower -= 0.0001;
                Revolver.setPower(Revolverpower);
            }

            if (revoval) {
                if (gamepad1.circle) {
                    revoval = false;
                    target += (8192 / 3) * 2;
                }
            }
            if (!revoval) {
                rl = -(Revolver.getCurrentPosition() - target) / Math.abs(Revolver.getCurrentPosition() - target);
                if (Math.abs(Revolver.getCurrentPosition() - target) > 1700) {
                    Revolver.setPower(Revolverpower * rl);
                } else if (Math.abs(Revolver.getCurrentPosition() - target) < 25) {
                    telemetry.addData("diff", Math.abs(Revolver.getCurrentPosition()));
                    Revolver.setPower(Revolverpower);
                    revoval = true;
                } else {
                    Revolver.setPower(Math.abs(Revolver.getCurrentPosition() - target) / (1700 / 0.5) * 0.3 * rl);//+min power
                }

            }
        }
            private void Telemetry() {

                telemetry.addData("Revolverpower", Revolverpower);

                telemetry.update();
            }

        }

