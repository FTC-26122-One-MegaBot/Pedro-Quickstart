package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Schiettest")
public class SchietTest extends LinearOpMode {
    private DcMotor arm;
    private CRServo passthrough1;
    private CRServo passthrough2;
    @Override
    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.get(DcMotor.class, "Arm");
        passthrough1 = hardwareMap.get(CRServo.class, "Passthrough 1");
        passthrough2 = hardwareMap.get(CRServo.class, "Passthrough 2");
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here

            while (opModeIsActive()) {

                flywheel();
                Passthrough1();
            }
        }
    }
    private void flywheel() {
        if (gamepad1.left_bumper) {
            arm.setPower(1);
        }else if (gamepad1.right_bumper) {
            arm.setPower(-1);
        }else {
            arm.setPower(0);
        }
    }
    private void Passthrough1() {
        if (gamepad1.left_trigger > 0.1 ) {
            passthrough1.setPower(1);
            passthrough2.setPower(1);
        } else if (gamepad1.right_trigger > 0.1) {
            passthrough1.setPower(-1);
            passthrough2.setPower(-1);
        } else  {
            passthrough1.setPower(0);
            passthrough2.setPower(0);
        }
    }
}