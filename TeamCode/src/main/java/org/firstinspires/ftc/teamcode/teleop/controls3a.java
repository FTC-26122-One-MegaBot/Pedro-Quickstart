package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(name = "controls3a")
public class controls3a extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double lastTime = 0;
    private double deltaTime = 0;
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotorEx Flywheel;
    private DcMotorEx Revolver;
    private DcMotorEx turretencoder;
    private CRServo turretturner;
    float speed = 0;
    public double integralsum = 0;
    double kp = 0.04;
    double ki = 0;
    double kd = 0;
    double preverror = 0;
    private CRServo Intake;
    private CRServo Shoot;
    double driveSlow = 0.5;
    double driveSpeed;
    double fx = 400;
    double fy = 400;
    double cx = 640 / 2.0; // = 320
    double cy = 480 / 2.0; // = 240
    boolean revoval = true;
    boolean vierkantwasingedrukt = false;
    boolean vierkantisingedrukt = false;
    double rx;
    int target;

    int Revolverpos = 0;
    int Revolverslot = 0;
    boolean squarepressed;
    boolean trianglepressed;
    ElapsedTime timer = new ElapsedTime();
    double Startpositierevolver = Revolver.getCurrentPosition();


    @Override
    public void runOpMode() throws InterruptedException {
        Intake = hardwareMap.get(CRServo.class, "Intake");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        Shoot = hardwareMap.get(CRServo.class, "Shoot");
        Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        Flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        turretturner = hardwareMap.get(CRServo.class, "Turret");
        Revolver = hardwareMap.get(DcMotorEx.class, "Revolver");
        Revolver.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); //BELANGRIJK
        turretencoder = hardwareMap.get(DcMotorEx.class, "turretencoder");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



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
            while (opModeIsActive()){
                Revolver.setPower(0.4);
                Revolver.setTargetPosition(Revolverpos);
                deltaTime = runtime.seconds() - lastTime;
                lastTime = runtime.seconds();
                Drive();
                Intake();
                revolve();
                shoot();
                flywheel();
                aim();
                AutoAim(tagProcessor);
                Telemetry();
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
        if (gamepad2.left_bumper){
            turretturner.setPower(1);
        } else if (gamepad2.right_bumper){
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
            if (gamepad2.right_stick_button) { // heb 'm nu op player 2 staan, lijkt me handiger? - ken
                if (Math.abs(tag.ftcPose.bearing) >= 2) {
                    double draaisnelheid = (tag.ftcPose.bearing) * -0.02; // constante moet nog worden getuned
                    Revolver.setVelocity(draaisnelheid); // PID?
;                   }
                }
            }
        }

        public void revolve(){
//        if (gamepad2.x){       //de inputs moeten nog op player 2 gezet worden
//            Revolver.setPower(0.4);
//        } else if (gamepad2.circle) {
//            if (Revolver.getCurrentPosition() - Startpositierevolver < 5440) { // 8190/3 plus minus 10
//                Revolver.setPower(0.4);
//            } else if (Revolver.getCurrentPosition() - Startpositierevolver > 5480) {
//                Revolver.setPower(-0.4);
//            }
//        } else if (gamepad2.triangle) {
//                if (Revolver.getCurrentPosition() - Startpositierevolver < -5440) {
//                    Revolver.setPower(0.4);
//                } else if (Revolver.getCurrentPosition() - Startpositierevolver > -5480) {
//                    Revolver.setPower(-0.4);
//                }
//        } else if (gamepad2.square) {
//            if (Revolver.getCurrentPosition() - Startpositierevolver < 20) {
//                Revolver.setPower(0.4);
//            } else if (Revolver.getCurrentPosition() - Startpositierevolver > - 20) {
//                Revolver.setPower(-0.4);
//            }
//        }
//        else if ((gamepad1.square && vierkantwasingedrukt) == false) {
//            vierkantisingedrukt = true;
//            vierkantwasingedrukt = true;
//        }
//        else if (vierkantwasingedrukt = true) {
//
//        }
        if (!gamepad1.square){
            squarepressed = false;
            }
        if (gamepad1.square && squarepressed == false) {
            squarepressed = true;
            Revolverslot = Revolverslot + 1;
            Revolverpos = Revolverslot * 5460;
        }
        if (!gamepad1.triangle){
            trianglepressed = false;
            }
        if (gamepad1.triangle && trianglepressed == false) {
            trianglepressed = true;
            Revolverslot = Revolverslot - 1;
            Revolverpos = Revolverslot * 5460;
            }

        /*if (revoval){
            if (gamepad1.circle){
                revoval = false;
                target += (8192/3)*2;
            }
        }
        if (!revoval){
            if (!gamepad1.circle){
                revoval = true;
            }
            if (Revolver.getCurrentPosition() < target) {
                Revolver.setPower(0.4);
            } else{
                Revolver.setPower(-0.4);
            }
        }
        if (Math.abs(Revolver.getCurrentPosition() - target) < 50){
            Revolver.setPower(0);
        }*/
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

        if (gamepad1.cross || gamepad2.cross) {
            speed = 0;
        }
        else {
            Flywheel.setVelocity(getpidspeed(speed, Flywheel.getVelocity()));

        }
    }
    private void shoot(){
        if (gamepad1.triangle){
            Shoot.setPower(1);
        } else if (gamepad1.square){
            Shoot.setPower(-1);
        }
        else {
            Shoot.setPower(0);
        }
    }

    private void Intake() {
        if (gamepad1.dpad_up){
            Intake.setPower(0.8);
        } else if (gamepad1.dpad_down){
            Intake.setPower(-0.8);
        } else {
            Intake.setPower(0);
        }
    }

    private void Telemetry() {
        telemetry.addData("rpm-flywheel", speed / 28 * 60); //reken ticks om naar rpm
        telemetry.addData("ticks-flywheel", speed);
        telemetry.addData("Flywheel", Flywheel.getVelocity());
        telemetry.addData("turret-turned", turretencoder.getCurrentPosition());
        telemetry.addData("revolver-turned", Revolver.getCurrentPosition());
        telemetry.addData("revoval", revoval);
        telemetry.addData("target", target);


        telemetry.addData("turn-factor", rx);

        telemetry.update();
    }

}