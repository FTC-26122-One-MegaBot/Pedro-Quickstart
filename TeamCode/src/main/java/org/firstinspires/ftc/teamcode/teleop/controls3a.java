package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(name = "Mainj")
public class Mainj extends LinearOpMode {
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
    double kp = 0.04;
    double ki = 0;
    double kd = 0;
    double preverror = 0;
    private DcMotor Intake;
    private CRServo Shoot;
    double driveSlow = 0.5;
    double driveSpeed;
    double fx = 400;
    double fy = 400;
    double cx = 640 / 2.0; // = 320
    double cy = 480 / 2.0; // = 240
    double rx;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        Shoot = hardwareMap.get(CRServo.class, "Shoot");
        kanonl = hardwareMap.get(DcMotorEx.class, "Flywheell");
        kanonl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); //BELANGRIJK
        kanonl.setDirection(DcMotorSimple.Direction.REVERSE);
        kanonr = hardwareMap.get(DcMotorEx.class, "Flywheelr");
        kanonr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); //BELANGRIJK
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);


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
                Intake();
                flywheels();
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
            driveSpeed = driveSlow;
        } else {
            driveSpeed = 1.0;
        }

        leftFront.setPower(frontLeftPower * driveSpeed);
        leftBack.setPower(backLeftPower * driveSpeed);
        rightFront.setPower(frontRightPower * driveSpeed);
        rightBack.setPower(backRightPower * driveSpeed);

    }
    public double getpidspeed(double target,double state){
        double error = target - state;
        integralsum += error * timer.seconds();
        double derivative = (error - preverror) / timer.seconds();
        timer.reset();
        preverror = error;
        return (kp * error) + (ki * integralsum) + (kd * derivative) + target;

    }
    public void flywheels(){
        if (gamepad1.left_trigger > 0.1){
            Shoot.setPower(-1);
        } else if (gamepad1.left_bumper){
            Shoot.setPower(1);
        } else{
            Shoot.setPower(0);
        }
        if (gamepad1.dpad_up || gamepad2.dpad_up){
            speed += 0.1;
        } else if (gamepad1.dpad_down || gamepad2.dpad_down){
            speed -= 0.1;
        } else if (gamepad2.right_trigger > 0.1){
            speed += 1;
        } else if (gamepad2.right_bumper){
            speed -= 1;
        } else if (gamepad2.left_trigger > 0.1){
            speed += 10;
        } else if (gamepad2.left_bumper){
            speed -= 10;
        }
        else if (gamepad1.dpad_left || gamepad2.dpad_left){
            speed = 2900 * 28 / 60; //snelheid van kleine driehoek
        }
        else if (gamepad1.dpad_right || gamepad2.dpad_right){
            speed = 2500 * 28 / 60 ; //snelheid van grote driehoek
        }

        if (gamepad1.cross || gamepad2.cross) {
            speed = 0;
        }
        if (gamepad1.circle || gamepad2.circle){
            speed = 1000;
        }
        else {
            kanonl.setVelocity(getpidspeed(speed, kanonl.getVelocity()));
            kanonr.setVelocity(getpidspeed(speed, kanonr.getVelocity()));
        }
    }

    private void Intake() {
        if (gamepad1.triangle){
            Intake.setPower(0.8);
        } else if (gamepad1.square){
            Intake.setPower(-0.8);
        } else {
            Intake.setPower(0);
        }
    }

    private void AutoAim(AprilTagProcessor tagProcessor) {
        // Controleer eerst of er ten minste één tag is gedetecteerd
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().iterator().next();

            // Alleen auto-aim als je op de rechter stick-knop drukt
            if (gamepad1.right_stick_button) {
                if (Math.abs(tag.ftcPose.bearing) >= 2) {
                    double draaisnelheid = (tag.ftcPose.bearing) * -0.02;

                        rightBack.setPower(-draaisnelheid);
                        rightFront.setPower(-draaisnelheid);
                        leftFront.setPower(draaisnelheid);
                        leftBack.setPower(draaisnelheid);
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
        telemetry.addData("rpm-flywheel", speed / 28 * 60); //reken ticks om naar rpm
        telemetry.addData("ticks-flywheel", speed);
        telemetry.addData("kanonleftticks", kanonl.getVelocity());
        telemetry.addData("kanonrightticks", kanonr.getVelocity());
        telemetry.addData("turn-factor", rx);

        telemetry.update();
    }

}