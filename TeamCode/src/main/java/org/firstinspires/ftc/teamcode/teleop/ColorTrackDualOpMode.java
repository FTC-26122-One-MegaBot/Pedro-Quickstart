package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Color Track Dual", group="Examples")
public class ColorTrackDualOpMode extends OpMode {

    OpenCvCamera webcam;
    ColorTrackPipeline pipeline;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Voeg hier je twee kleuren toe (HSV)
        List<Scalar> colors = new ArrayList<>();
        colors.add(new Scalar(150, 120, 150)); // eerste kleur (bijv. blauwachtig)
        colors.add(new Scalar(79, 254, 130));  // tweede kleur (bijv. groenachtig)

        pipeline = new ColorTrackPipeline(colors);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });
    }

    @Override
    public void loop() {
        telemetry.addLine("Tracking 2 colors...");
        telemetry.addData("Object 1 (H=150)", "X=%d, Y=%d", pipeline.objectX[0], pipeline.objectY[0]);
        telemetry.addData("Object 2 (H=79)", "X=%d, Y=%d", pipeline.objectX[1], pipeline.objectY[1]);
        telemetry.update();
    }

    // ==========================
    // PIPELINE CODE
    // ==========================
    class ColorTrackPipeline extends OpenCvPipeline {

        private List<Scalar> targetColors;  // Lijst met kleuren om te tracken
        public int[] objectX;
        public int[] objectY;

        public ColorTrackPipeline(List<Scalar> colors) {
            this.targetColors = colors;
            objectX = new int[colors.size()];
            objectY = new int[colors.size()];
            for(int i = 0; i < colors.size(); i++){
                objectX[i] = -1;
                objectY[i] = -1;
            }
        }

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            for(int i = 0; i < targetColors.size(); i++){
                Scalar targetColor = targetColors.get(i);

                Scalar lower = new Scalar(targetColor.val[0]-10, 100, 100);
                Scalar upper = new Scalar(targetColor.val[0]+10, 255, 255);

                Mat mask = new Mat();
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
                        // Gebruik verschillende kleuren voor visualisatie
                        Scalar drawColor = (i == 0) ? new Scalar(0,255,0) : new Scalar(0,0,255);
                        Imgproc.rectangle(input, bestRect, drawColor, 2);
                        objectX[i] = bestRect.x + bestRect.width/2;
                        objectY[i] = bestRect.y + bestRect.height/2;
                    } else {
                        objectX[i] = -1;
                        objectY[i] = -1;
                    }
                } else {
                    objectX[i] = -1;
                    objectY[i] = -1;
                }

                mask.release();
                hierarchy.release();
            }

            hsv.release();
            return input;
        }
    }
}
