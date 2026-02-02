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


@TeleOp(name = "Main")
public class Main extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double lastTime = 0;
    private double deltaTime = 0;
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotorEx FlyWheel;
    private DcMotor Intake;
    private Servo Shoot;
    double driveFast = 1.0;
    double driveSpeed;
    double FlywheelSpeed = 0;
    double fx = 400;
    double fy = 400;
    double cx = 640 / 2.0; // = 320
    double cy = 480 / 2.0; // = 240
    double rx;

    @Override
    public void runOpMode() throws InterruptedException {
        FlyWheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        Shoot = hardwareMap.get(Servo.class, "Shoot");
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        FlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(fx, fy, cx, cy)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive() && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING){

                deltaTime = runtime.seconds() - lastTime;
                lastTime = runtime.seconds();
                Drive();
                FlyWheel();
                Shoot();
                Intake();
                AutoAim(tagProcessor);
                Telemetry(tagProcessor);
            }
        }


    }
    private void Drive() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        if (gamepad1.right_bumper) {
            driveSpeed = driveFast;
        } else {
            driveSpeed = 0.5;
        }

        leftFront.setPower(frontLeftPower * driveSpeed);
        leftBack.setPower(backLeftPower * driveSpeed);
        rightFront.setPower(frontRightPower * driveSpeed);
        rightBack.setPower(backRightPower * driveSpeed);

    }
    private void FlyWheel() {
        if (gamepad1.right_trigger > 0.1){
            FlyWheel.setVelocity(FlywheelSpeed);
        } else {
            FlyWheel.setPower(0);
        }
        if (gamepad1.dpad_up){
            FlywheelSpeed += 280 * deltaTime;
        } else if (gamepad1.dpad_down){
            FlywheelSpeed -= 280 * deltaTime;
        }
    }
    private void Intake() {
        if (gamepad1.x){
            Intake.setPower(0.8);
        } else if (gamepad1.circle){
            Intake.setPower(-0.8);
        } else {
            Intake.setPower(0);
        }
    }
    private void Shoot(){
        if (gamepad1.square){
            Shoot.setPosition(1.4);
        } else if (gamepad1.triangle){
            Shoot.setPosition(0.5);
        }

    }
    private void AutoAim(AprilTagProcessor tagProcessor) {
        // Controleer eerst of er ten minste één tag is gedetecteerd
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().iterator().next();

            // Alleen auto-aim als je op de rechter stick-knop drukt
            if (gamepad1.right_stick_button) {
                if (Math.abs(tag.ftcPose.bearing) >= 2) {
                    double draaisnelheid = (tag.ftcPose.bearing) * 0.2;

                        rightBack.setPower(draaisnelheid);
                        rightFront.setPower(draaisnelheid);
                        leftFront.setPower(-draaisnelheid);
                        leftBack.setPower(-draaisnelheid);
                }
            }
        }

    }
    private void Telemetry(AprilTagProcessor tagProcessor) {
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().iterator().next();
            try {
                telemetry.addData("x-distance", tag.ftcPose.x);
                telemetry.addData("y-distance", tag.ftcPose.y);
                telemetry.addData("z-distance", tag.ftcPose.z);
                telemetry.addData("tot-distance", tag.ftcPose.range);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("bearing", tag.ftcPose.bearing);
                telemetry.addData("ID", tag.id);
                if (tag.id == 21){
                    telemetry.addData("Patern","🟩🟪🟪");
                } else if (tag.id == 22) {
                    telemetry.addData("Patern","🟪🟩🟪");
                } else if (tag.id == 23) {
                    telemetry.addData("Patern", "🟪🟪🟩");

                }
            } catch (NullPointerException e) {
                // Foo
            }
        }
        telemetry.addData("flywheel-target-velocity", FlywheelSpeed);
        telemetry.addData("flywheel-speed", FlyWheel.getVelocity());
        telemetry.addData("turn-factor", rx);
        telemetry.update();
    }

}