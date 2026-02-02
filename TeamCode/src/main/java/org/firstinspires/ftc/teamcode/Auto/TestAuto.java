package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TestAuto", group="Linear OpMode")
public class TestAuto extends LinearOpMode {
    // Initialize motor variables
    ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    @Override
    public void runOpMode() {
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive() && runtime.seconds() <= 2) {
                leftBack.setPower(0.4);
                leftFront.setPower(0.4);
                rightBack.setPower(0.4);
                rightFront.setPower(0.4);
            }

            while (opModeIsActive() && runtime.seconds() > 2) {
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
            }

        }
    }
}
