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

@TeleOp(name = "ManualMotorTest")
public class ManualMotorTest extends LinearOpMode {
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    @Override
    public void runOpMode() throws InterruptedException {
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad1.square) {
                    leftBack.setPower(0.3);
                    telemetry.addLine("leftBack");
                    telemetry.update();
                } else if (gamepad1.triangle) {
                    leftFront.setPower(0.3);
                    telemetry.addLine("leftFront");
                    telemetry.update();
                } else if (gamepad1.circle) {
                    rightFront.setPower(0.3);
                    telemetry.addLine("rightFront");
                    telemetry.update();
                } else if (gamepad1.cross) {
                    rightBack.setPower(0.3);
                    telemetry.addLine("RightBack");
                    telemetry.update();
                } else{
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                }
            }
        }
    }
}
