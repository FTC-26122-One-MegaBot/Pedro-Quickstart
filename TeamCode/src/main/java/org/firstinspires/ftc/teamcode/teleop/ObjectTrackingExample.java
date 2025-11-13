package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.MatOfPoint;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="ObjectTrackingExample", group="Examples")
public class ObjectTrackingExample extends OpMode {

    OpenCvCamera webcam;
    ObjectTrackingPipeline pipeline;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new ObjectTrackingPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    @Override
    public void loop() {
        telemetry.addData("Object X", pipeline.objectX);
        telemetry.addData("Object Y", pipeline.objectY);
        telemetry.update();
    }

    // Pipeline voor object tracking
    class ObjectTrackingPipeline extends OpenCvPipeline {

        public int objectX = -1;
        public int objectY = -1;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Mat mask = new Mat();

            // Converteer naar HSV
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Stel kleurbereik in (bijv. rood)
            Scalar lower = new Scalar(0, 100, 100);   // H, S, V
            Scalar upper = new Scalar(10, 255, 255);

            Core.inRange(hsv, lower, upper, mask);

            // Vind contouren
            java.util.List<MatOfPoint> contours = new java.util.ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) {
                // Grootste contour kiezen
                double maxArea = 0;
                Rect bestRect = null;
                for (MatOfPoint contour : contours) {
                    Rect rect = Imgproc.boundingRect(contour);
                    double area = rect.area();
                    if (area > maxArea) {
                        maxArea = area;
                        bestRect = rect;
                    }
                }

                if (bestRect != null) {
                    // Teken rechthoek op scherm
                    Imgproc.rectangle(input, bestRect, new Scalar(0, 255, 0), 2);
                    objectX = bestRect.x + bestRect.width / 2;
                    objectY = bestRect.y + bestRect.height / 2;
                }
            }

            return input;
        }
    }
}
