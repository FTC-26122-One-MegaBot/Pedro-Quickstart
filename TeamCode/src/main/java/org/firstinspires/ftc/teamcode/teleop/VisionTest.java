package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//TeleOp
public class VisionTest extends LinearOpMode {

    double fx = 400;
    double fy = 400;
    double cx = 640 / 2.0; // = 320
    double cy = 480 / 2.0; // = 240

    @Override
    public void runOpMode() throws InterruptedException{

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

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {}

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);

        waitForStart();
        if (opModeIsActive()){

            while (opModeIsActive()){

                if (!tagProcessor.getDetections().isEmpty()) {
                    AprilTagDetection tag = tagProcessor.getDetections().iterator().next();
                    try {
                        telemetry.addData("x-distance", tag.ftcPose.x);
                        telemetry.addData("y-distance", tag.ftcPose.y);
                        telemetry.addData("z-distance", tag.ftcPose.z);
                        telemetry.addData("tot-distance", tag.ftcPose.range);
                        telemetry.addData("yaw", tag.ftcPose.yaw);
                        telemetry.addData("pitch", tag.ftcPose.pitch);
                        telemetry.addData("ID", tag.id);
                        if (tag.id == 21) {
                            telemetry.addData("Patern", "🟩🟪🟪");
                        } else if (tag.id == 22) {
                            telemetry.addData("Patern", "🟪🟩🟪");
                        } else if (tag.id == 23) {
                            telemetry.addData("Patern", "🟪🟪🟩");

                        }
                    }catch (NullPointerException e) {
                        // Foo
                    }
                    telemetry.update();
                }
            }
        }
    }
}
