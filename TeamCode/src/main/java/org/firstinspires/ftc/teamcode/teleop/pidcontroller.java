package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="pidcontroller")
public class pidcontroller extends LinearOpMode {
    private DcMotorEx kanonl;
    private DcMotorEx kanonr;
    //gebruik "private Servo 'servonaam'" voor een servomotor
    float speed = 0;
    public double integralsum = 0;
    double kp = 0;
    double ki = 0;
    double kd = 0;
    double preverror = 0;
    int pick = 0;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime tuningtimer = new ElapsedTime();
    double tuningpower = 1;
    @Override
    public void runOpMode() {
        kanonl = hardwareMap.get(DcMotorEx.class, "Flywheell");
        kanonl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); //BELANGRIJK
        kanonl.setDirection(DcMotorSimple.Direction.REVERSE);
        kanonr = hardwareMap.get(DcMotorEx.class, "Flywheelr");
        kanonr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); //BELANGRIJK


        waitForStart();
        while(opModeIsActive()) {
            flywheels();
            tuning();
            controller();
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
    public void flywheels(){
        if (gamepad1.dpad_up){
            speed += 0.1;
        } else if (gamepad1.dpad_down){
            speed -= 0.1;
        }
        else if (gamepad1.dpad_left){
            speed = 2900 * 28 / 60; //snelheid van kleine driehoek
        }
        else if (gamepad1.dpad_right){
            speed = 2700 * 28 / 60 ; //snelheid van grote driehoek
        }

        if (gamepad1.cross) {
            speed = 0;
        }
        if (gamepad1.circle){
            speed = 1000;
        }
        else {
            kanonl.setVelocity(getpidspeed(speed, kanonl.getVelocity()));
            kanonr.setVelocity(getpidspeed(speed, kanonr.getVelocity()));
        }
    }

    public void tuning(){
        if (tuningtimer.seconds() > 0.4) {
            if (gamepad1.left_bumper){
                tuningpower *= 10;
                if (tuningpower == 100){
                    tuningpower = 0.001;
                }
            }
            if (gamepad1.left_trigger > 0.4){
                pick += 1;
                if (pick == 4){
                    pick = 0;
                }
            }
            switch (pick){
                case 1:
                    if (gamepad1.square){
                        kp += tuningpower;
                    }
                    else if (gamepad1.triangle){
                        kp -= tuningpower;
                    }
                    break;
                case 2:
                    if (gamepad1.square){
                        ki += tuningpower;
                    }
                    else if (gamepad1.triangle){
                        ki -= tuningpower;
                    }
                    break;
                case 3:
                    if (gamepad1.square){
                        kd += tuningpower;
                    }
                    else if (gamepad1.triangle){
                        kd -= tuningpower;
                    }
                    break;
            }
            timer.reset();
        }
    }
    public void controller() {
        double strafe = gamepad1.left_stick_x;
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double joystickrechtsy = gamepad1.right_stick_y;
        boolean knopa = gamepad1.a;
        boolean knopb = gamepad1.b;
        boolean knopx = gamepad1.x;
        boolean knopy = gamepad1.y;
        boolean dpadup = gamepad1.dpad_up;
        boolean dpadright = gamepad1.dpad_right;
        boolean dpaddown = gamepad1.dpad_down;
        boolean dpadleft = gamepad1.dpad_left;
        boolean leftbumper = gamepad1.left_bumper;
        boolean rightbumper = gamepad1.right_bumper;
        float lefttrigger = gamepad1.left_trigger;
        float righttrigger = gamepad1.right_trigger;
        boolean knopstart = gamepad1.start;
        boolean knopstop = gamepad1.back;
        telemetry.addData("rpm-flywheel", speed / 28 * 60); //reken ticks om naar rpm
        telemetry.addData("ticks-flywheel", speed);
        telemetry.addData("kanonleftticks", kanonl.getVelocity());
        telemetry.addData("kanonrightticks", kanonr.getVelocity());
        telemetry.addData("kp", kp);
        telemetry.addData("ki", ki);
        telemetry.addData("kd", kd);
        telemetry.addData("pick", pick);
        telemetry.addData("tuningpower",tuningpower);
        telemetry.update();

    }



}
