package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "AprilTag Auto WORKING")
public class AprilTagAutoWorking extends LinearOpMode {

    /* ================= MOTORS ================= */

    DcMotor rightFront, leftFront, leftBack, rightBack;

    /* ================= VISION ================= */

    VisionPortal visionPortal;
    AprilTagProcessor tagProcessor;

    /* ================= CONSTANTS ================= */

    static final int TARGET_TAG_ID = 20;

    static final double TARGET_DISTANCE_CM = 40.0;
    static final double DIST_TOLERANCE = 2.0;
    static final double BEARING_TOLERANCE = 2.0;

    // Camera intrinsics (zelfde als je TeleOp)
    double fx = 400;
    double fy = 400;
    double cx = 640 / 2.0;
    double cy = 480 / 2.0;

    @Override
    public void runOpMode() {

        /* ========== MOTORS ========== */

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftRear");
        rightBack  = hardwareMap.get(DcMotor.class, "rightRear");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        /* ========== APRILTAG ========== */

        tagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(fx, fy, cx, cy)
                .setDrawTagID(true)
                .setDrawAxes(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        telemetry.addLine("Initialized");
        telemetry.update();

        /* ========== WAIT FOR START ========== */

        waitForStart();

        /* ========== MAIN AUTO LOOP ========== */

        while (opModeIsActive()
                && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {

            if (tagProcessor.getDetections().isEmpty()) {
                telemetry.addLine("No AprilTag");
                telemetry.update();
                stopDrive();
                continue;
            }

            AprilTagDetection tag = tagProcessor.getDetections().get(0);

            if (tag.id != TARGET_TAG_ID) {
                telemetry.addData("Wrong tag", tag.id);
                telemetry.update();
                stopDrive();
                continue;
            }

            // FTC SDK values
            double distanceCm = tag.ftcPose.range * 100.0;
            double bearing = tag.ftcPose.bearing;

            double drive = 0;
            double turn = 0;

            double distError = distanceCm - TARGET_DISTANCE_CM;

            if (Math.abs(distError) > DIST_TOLERANCE) {
                drive = distError * 0.02;
            }

            if (Math.abs(bearing) > BEARING_TOLERANCE) {
                turn = bearing * -0.02;
            }

            drive = clamp(drive, -0.4, 0.4);
            turn = clamp(turn, -0.3, 0.3);

            // Tank-style correction (simpel & stabiel)
            leftFront.setPower(drive + turn);
            leftBack.setPower(drive + turn);
            rightFront.setPower(drive - turn);
            rightBack.setPower(drive - turn);

            telemetry.addData("Distance (cm)", distanceCm);
            telemetry.addData("Bearing", bearing);
            telemetry.update();

            // STOPCOND
            if (Math.abs(distError) <= DIST_TOLERANCE
                    && Math.abs(bearing) <= BEARING_TOLERANCE) {

                stopDrive();
                telemetry.addLine("ALIGNED ✅");
                telemetry.update();
                sleep(500);
                break;
            }
        }

        stopDrive();
    }

    /* ================= HELPERS ================= */

    private void stopDrive() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
