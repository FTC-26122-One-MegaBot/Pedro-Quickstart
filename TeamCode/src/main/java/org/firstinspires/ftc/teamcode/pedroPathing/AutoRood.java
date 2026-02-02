package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Auto Rood")
public class AutoRood extends OpMode {
    private DcMotor Intake;
    private CRServo Shoot;
    private DcMotorEx kanonl;
    private DcMotorEx kanonr;


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private final Pose startPose = new Pose(111, 135, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(104, 110, Math.toRadians(55)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose prePickup1 = new Pose(92, 85, Math.toRadians(180));
    private final Pose Pickup1 = new Pose(115, 85, Math.toRadians(180));
    private final Pose prePickup2 = new Pose(96, 60, Math.toRadians(180));
    private final Pose Pickup2 = new Pose(126, 60, Math.toRadians(180));
    private final Pose prePickup3 = new Pose(96, 36, Math.toRadians(180));
    private final Pose Pickup3 = new Pose(126, 36, Math.toRadians(180));

    //    private Path scorePreload;
    private PathChain scorePreload, drivePickup1, grabPickup1, scorePickup1, drivePickup2, grabPickup2, scorePickup2, drivePickup3, grabPickup3, scorePickup3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        drivePickup1 = follower.pathBuilder()
                .addPath(new BezierLine (scorePose, prePickup1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup1.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(prePickup1, Pickup1))
                .setLinearHeadingInterpolation(prePickup1.getHeading(), Pickup1.getHeading())
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(Pickup1, scorePose))
                .setLinearHeadingInterpolation(Pickup1.getHeading(), scorePose.getHeading())
                .build();
        drivePickup2 = follower.pathBuilder()
                .addPath(new BezierLine (scorePose, prePickup2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup2.getHeading())
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(prePickup2, Pickup2))
                .setLinearHeadingInterpolation(prePickup2.getHeading(), Pickup2.getHeading())
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(Pickup2, scorePose))
                .setLinearHeadingInterpolation(Pickup2.getHeading(), scorePose.getHeading())
                .build();
        drivePickup3 = follower.pathBuilder()
                .addPath(new BezierLine (scorePose, prePickup3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup3.getHeading())
                .build();
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(prePickup3, Pickup3))
                .setLinearHeadingInterpolation(prePickup3.getHeading(), Pickup3.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(Pickup3, scorePose))
                .setLinearHeadingInterpolation(Pickup3.getHeading(), scorePose.getHeading())
                .build();


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, 1, true);
                    setPathState(1);

                break;
            case 1:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Intake.setPower(0.8);
                    Shoot.setPower(-1);
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(drivePickup1, 1, false);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(drivePickup1, false);
                    Intake.setPower(0.8);
                    Shoot.setPower(0);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1, true);
                    Intake.setPower(0);
                    Shoot.setPower(0);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1, false);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Intake.setPower(0.8);
                    Shoot.setPower(-1);
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(drivePickup1, false);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabPickup1, true);
                    setPathState(-1);

                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(drivePickup3, false);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3, false);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    if (!follower.isBusy()) {
                        Shoot.setPower(-1);
                        Sleep(500L);
                        Shoot.setPower(0);
                        setPathState(10);
                    }
                }
                break;
            case 10:
                if (!follower.isBusy()){
                    follower.followPath(drivePickup2,false);
                }
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        kanonl.setVelocity(2350 * 28 / 60);
        kanonr.setVelocity(2350 * 28 / 60);
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        Shoot = hardwareMap.get(CRServo.class, "Shoot");

        kanonl = hardwareMap.get(DcMotorEx.class, "Flywheell");
        kanonl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); //BELANGRIJK
        kanonl.setDirection(DcMotorSimple.Direction.REVERSE);

        kanonr = hardwareMap.get(DcMotorEx.class, "Flywheelr");
        kanonr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); //BELANGRIJK

        pathState = 0;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }

    private void Sleep(long time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}