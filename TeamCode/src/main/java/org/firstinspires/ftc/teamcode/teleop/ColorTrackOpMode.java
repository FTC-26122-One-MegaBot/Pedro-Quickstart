package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.opencv.core.MatOfPoint;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;

import java.util.ArrayList;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Color Track", group="Examples")
public class ColorTrackOpMode extends OpMode {

    OpenCvCamera webcam;
    ColorTrackPipeline pipeline;

    // Vervang deze HSV-waarden door de gemeten waarden
    Scalar targetColorHSV = new Scalar(150, 120, 120); // H, S, V

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new ColorTrackPipeline(targetColorHSV);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
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

    class ColorTrackPipeline extends OpenCvPipeline {

        private Scalar targetColor;
        public int objectX = -1;
        public int objectY = -1;

        public ColorTrackPipeline(Scalar targetColor) {
            this.targetColor = targetColor;
        }

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Mat mask = new Mat();

            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Scalar lower = new Scalar(targetColor.val[0]-10, 100, 100);
            Scalar upper = new Scalar(targetColor.val[0]+10, 255, 255);

            Core.inRange(hsv, lower, upper, mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if(!contours.isEmpty()){
                double maxArea = 0;
                Rect bestRect = null;
                for(MatOfPoint contour : contours){
                    Rect rect = Imgproc.boundingRect(contour);
                    double area = rect.area();
                    if(area > maxArea){
                        maxArea = area;
                        bestRect = rect;
                    }
                }

                if(bestRect != null){
                    Imgproc.rectangle(input, bestRect, new Scalar(0,255,0), 2);
                    objectX = bestRect.x + bestRect.width/2;
                    objectY = bestRect.y + bestRect.height/2;
                }
            } else {
                objectX = -1;
                objectY = -1;
            }

            hsv.release();
            mask.release();
            hierarchy.release();

            return input;
        }
    }
}
