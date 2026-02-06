package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(name = "j52")
public class j52 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double lastTime = 0;
    private double deltaTime = 0;
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotorEx FlywheelBoven;
    private DcMotorEx FlywheelOnder;
    private DcMotorEx Revolver;
    private CRServo turretturner;
    float speed = 0;
    public double integralsum = 0;
    double kp = 0.04;
    double ki = 0;
    double kd = 0;
    double preverror = 0;
    private DcMotorEx Intake; // moet core hex worden!!
    private CRServo Shoot;
    double driveSlow = 0.5;
    double driveSpeed;
    double fx = 400;
    double fy = 400;
    double cx = 640 / 2.0; // = 320
    double cy = 480 / 2.0; // = 240
    boolean revoval = true;
    double rx;
    int target;
    int RevolverTpos = 0;
    int Revolverslot = 0;
    boolean crosspressed;
    boolean circlepressed;
    int rl;
    boolean rem;
    double power;
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        Shoot = hardwareMap.get(CRServo.class, "Shoot");
        FlywheelBoven = hardwareMap.get(DcMotorEx.class, "FlywheelBoven");
        FlywheelBoven.setDirection(DcMotorSimple.Direction.FORWARD);
        FlywheelBoven.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FlywheelOnder = hardwareMap.get(DcMotorEx.class, "FlywheelOnder");
        FlywheelOnder.setDirection(DcMotorSimple.Direction.FORWARD);
        FlywheelOnder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretturner = hardwareMap.get(CRServo.class, "Turret");
        Revolver = hardwareMap.get(DcMotorEx.class, "Revolver");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
//        Revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



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
            while (opModeIsActive()  && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING){
                deltaTime = runtime.seconds() - lastTime;
                lastTime = runtime.seconds();
                Drive();
                Intake();
                revolve();
                shoot();
                flywheel();
                aim();
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
    private void aim(){
        if (gamepad1.left_bumper){
            turretturner.setPower(1);
        } else if (gamepad1.left_trigger > 0.4){
            turretturner.setPower(-1);
        } else{
            turretturner.setPower(0);
        }
    };
    private void AutoAim(AprilTagProcessor tagProcessor) {
        // Controleer eerst of er ten minste één tag is gedetecteerd
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().iterator().next();

            // Alleen auto-aim als je op de rechter stick-knop drukt
            if (gamepad2.dpad_up) { // heb 'm nu op player 2 staan, lijkt me handiger? - ken
                if (Math.abs(tag.ftcPose.pitch) >= 2) {
                    double draaisnelheid = (tag.ftcPose.pitch) * -0.02; // constante moet nog worden getuned
                    turretturner.setPower(draaisnelheid); // PID?
                   }
                }
            }
        }

        public void revolve(){
        if (revoval){
            if (gamepad1.circle){
                revoval = false;
                target += (8192/3)*2;
            }
        }
       if (!revoval){
           try {
               rl = -(Revolver.getCurrentPosition() - target) / Math.abs(Revolver.getCurrentPosition() - target);
           } catch (ArithmeticException e){
               rl = 0;
           }
           /*if (Math.abs(Revolver.getCurrentPosition() - target) > 1600){
               Revolver.setPower(0.6 * rl);
           } else if ( Math.abs(Revolver.getCurrentPosition() - target) < 25) {
               telemetry.addData("diff",Math.abs(Revolver.getCurrentPosition()));
               Revolver.setPower(0);
               revoval = true;
           } else {
               Revolver.setPower(Math.abs((Revolver.getCurrentPosition() - target) * 0.000231 +0.23) * rl);//+min power
           }*/
           power = ((Math.abs(Revolver.getCurrentPosition()-target) * (0.4/((8192/3.0)*2)))+0.06)  * rl; // b was 0.06
           if (power > 0.5){
               power = 0.5;
           } else if (power < -0.5){
               power = -0.5;
           }
           Revolver.setPower(power);
           if (Math.abs(Revolver.getCurrentPosition()- target) < 50) {
               Revolver.setPower(0);
               revoval = true;
           }

        }
    }
    public double getpidspeed(double target,double state){
        double error = target - state;
        integralsum += error * timer.seconds();
        double derivative = (error - preverror) / timer.seconds();
        timer.reset();
        preverror = error;
        return (kp * error) + (ki * integralsum) + (kd * derivative) + target;

    }
    public void flywheel(){
        if (gamepad1.dpad_left || gamepad2.dpad_left){
            speed = 2900 * 28 / 60; //snelheid van kleine driehoek
        }
        else if (gamepad1.dpad_right || gamepad2.dpad_right){
            speed = 3500 * 28 / 60 ; //snelheid van grote driehoek
        }

        if (gamepad1.cross) {
            rem = true;
        }
        if (rem) {
            speed -= 3;
            FlywheelBoven.setVelocity(getpidspeed(speed, FlywheelBoven.getVelocity()));
            FlywheelOnder.setVelocity(getpidspeed(speed, FlywheelOnder.getVelocity()));
            if (speed < 400){
                speed = 0;
                rem = false;
            }
        } else {
            FlywheelBoven.setVelocity(getpidspeed(speed, FlywheelBoven.getVelocity()));
            FlywheelOnder.setVelocity(getpidspeed(speed, FlywheelOnder.getVelocity()));

        }
    }
    private void shoot(){
        if (gamepad1.right_bumper){
            Shoot.setPower(1);
        } else if (gamepad1.right_trigger > 0.4){
            Shoot.setPower(-1);
        }
        else {
            Shoot.setPower(0);
        }
    }

    private void Intake() {
        if (gamepad1.square){
            Intake.setPower(0.8);
        } else if (gamepad1.triangle){
            Intake.setPower(-0.8);
        } else {
            Intake.setPower(0);
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
        telemetry.addData("RevolverTPos", RevolverTpos );
        telemetry.addData("RevolverSlot", Revolverslot);
        telemetry.addData("Revolvertarget", Revolver.getTargetPosition());
        telemetry.addData("rpm-flywheel", speed / 28 * 60); //reken ticks om naar rpm
        telemetry.addData("ticks-flywheel", speed);
        telemetry.addData("Flywheel", FlywheelBoven.getVelocity());
        telemetry.addData("turret-turned", Intake.getCurrentPosition());
        telemetry.addData("revolver-turned", Revolver.getCurrentPosition());
        telemetry.addData("revoval", revoval);
        telemetry.addData("target", target);
        telemetry.addData("diff",Math.abs(Revolver.getCurrentPosition()- target));
        telemetry.addData("power", power);


        telemetry.addData("turn-factor", rx);

        telemetry.update();
    }

}