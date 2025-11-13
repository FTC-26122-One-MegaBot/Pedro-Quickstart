package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Color Measure", group="Examples")
public class ColorMeasureOpMode extends OpMode {

    OpenCvCamera webcam;
    ColorMeasurePipeline pipeline;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new ColorMeasurePipeline();
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
        Scalar color = pipeline.getAverageHSV();
        telemetry.addData("Measured H", (int)color.val[0]);
        telemetry.addData("Measured S", (int)color.val[1]);
        telemetry.addData("Measured V", (int)color.val[2]);
        telemetry.update();
    }

    class ColorMeasurePipeline extends OpenCvPipeline {
        private Mat currentFrame = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(currentFrame);
            return input;
        }

        public Scalar getAverageHSV() {
            if(currentFrame.empty()) return new Scalar(0,0,0);

            Mat hsv = new Mat();
            Imgproc.cvtColor(currentFrame, hsv, Imgproc.COLOR_RGB2HSV);

            int cx = hsv.cols()/2;
            int cy = hsv.rows()/2;
            int size = 25;
            int x1 = Math.max(cx-size, 0);
            int y1 = Math.max(cy-size, 0);
            int x2 = Math.min(cx+size, hsv.cols());
            int y2 = Math.min(cy+size, hsv.rows());

            Mat roi = hsv.submat(new Rect(x1,y1,x2-x1,y2-y1));
            Scalar avgHSV = org.opencv.core.Core.mean(roi);

            roi.release();
            hsv.release();
            return avgHSV;
        }
    }
}
