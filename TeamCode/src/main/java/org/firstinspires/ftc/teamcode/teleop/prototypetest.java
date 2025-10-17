package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(name = "prototypetest", group = "TeleOp")
public class prototypetest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double lastTime = 0;
    private double deltaTime = 0;
    private DcMotorEx right;
    private DcMotorEx left;
    private DcMotor flywheel;
    double driveFast = 1.0;
    double driveSpeed;
    double FlywheelSpeed = 0;
    double fx = 400;
    double fy = 400;
    double cx = 640 / 2.0; // = 320
    double cy = 480 / 2.0; // = 240
    double speed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(DcMotorEx.class, "left");
        right = hardwareMap.get(DcMotorEx.class, "right");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                deltaTime = runtime.seconds() - lastTime;
                lastTime = runtime.seconds();
                test();
            }
        }


    }

    private void test() {

        if (gamepad1.dpad_up) {
            speed += 1;
        }
        else if (gamepad1.dpad_down){
            speed -= 1;
        }
        else if (gamepad1.dpad_left){
            speed -= 10;
        }
        else if (gamepad1.dpad_right) {
            speed -= 10;
        }
        else if (gamepad1.triangle){
            speed = 0;
        }
        if (speed > 6000 * 28 / 60){
            speed = 6000 * 28 / 60;
        }
        else if (speed < -6000 * 28 / 60){
            speed =  -6000 * 28 / 60;
        }
        right.setVelocity(speed);
        left.setVelocity(speed);
        telemetry.addData("target rpm", speed / 28 * 60);
        telemetry.addData("rpm_left", left.getVelocity() / 28 * 60);
        telemetry.addData("rpm_right", right.getVelocity() / 28 * 60);
        telemetry.update();
    }
}