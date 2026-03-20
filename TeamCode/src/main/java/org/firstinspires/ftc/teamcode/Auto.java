package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Auto_Comp", group="Auto")
public class Auto extends OpMode {

    // ================= DRIVE =================
    private DcMotor frontLeft, backLeft, frontRight, backRight;

    // ================= MECHANISMS =================
    private DcMotor intakeMotor;
    private DcMotor shooterLeft, shooterRight;
    private Servo feederServo;

    private final double SERVO_CLOSED = 0.0;
    private final double SERVO_OPEN = 0.5;

    // ================= PATHING =================
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private PathChain scorePreload;
    private PathChain grabPickup1, scorePickup1;
    private PathChain grabPickup2, scorePickup2;
    private PathChain grabPickup3, scorePickup3;

    private int pathState = 0;

    // ================= POSES =================
    private final Pose startPose = new Pose(20.79, 122.89, Math.toRadians(-35));
    private final Pose scorePose = new Pose(59.61, 84.00, Math.toRadians(135));

    private final Pose pickup1Pose = new Pose(24.11, 84.30, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(18.98, 60.16, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(17.16, 35.84, Math.toRadians(180));

    // ================= CONTROL =================
    private boolean hasShot = false;
    private int shootStep = 0;
    private double shootStartTime = 0;

    // ================= INIT =================
    @Override
    public void init() {

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
        feederServo = hardwareMap.get(Servo.class, "feederServo");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        feederServo.setPosition(SERVO_CLOSED);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();

        // ================= INTAKE =================
        if ((pathState == 1 || pathState == 3 || pathState == 5) && follower.isBusy()) {
            intakeMotor.setPower(0.8);
        } else {
            intakeMotor.setPower(0);
        }

        // ================= SHOOTER =================
        if ((pathState == 2 || pathState == 4 || pathState == 6 || pathState == 7)
                && !follower.isBusy()
                && pathTimer.getElapsedTimeSeconds() > 2.0) {

            shooterLeft.setPower(0.7);
            shooterRight.setPower(0.7);

            if (!hasShot) {
                shootThreeRingsNonBlocking();
                if (shootStep == 0) {
                    hasShot = true;
                }
            }

        } else {
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
        }

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void stop() {
        stopRobot();
    }

    // ================= PATHS =================
    private void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(135))
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup2Pose,
                        new Pose(43.85, 55.66),
                        scorePose))
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup3Pose,
                        new Pose(60.65, 29.43),
                        scorePose))
                .build();
    }

    // ================= STATE MACHINE =================
    private void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    setPathState(-1);
                }
                break;
        }
    }

    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();

        if (state == 2 || state == 4 || state == 6 || state == 7) {
            hasShot = false;
            shootStep = 0;
        }
    }

    // ================= SHOOTING =================
    private void shootThreeRingsNonBlocking() {

        double currentTime = opmodeTimer.getElapsedTimeSeconds();

        switch (shootStep) {
            case 0:
                feederServo.setPosition(SERVO_OPEN);
                shootStartTime = currentTime;
                shootStep++;
                break;

            case 1:
                if (currentTime - shootStartTime > 0.2) {
                    feederServo.setPosition(SERVO_CLOSED);
                    shootStartTime = currentTime;
                    shootStep++;
                }
                break;

            case 2:
                if (currentTime - shootStartTime > 0.2) {
                    feederServo.setPosition(SERVO_OPEN);
                    shootStartTime = currentTime;
                    shootStep++;
                }
                break;

            case 3:
                if (currentTime - shootStartTime > 0.2) {
                    feederServo.setPosition(SERVO_CLOSED);
                    shootStartTime = currentTime;
                    shootStep++;
                }
                break;

            case 4:
                if (currentTime - shootStartTime > 0.2) {
                    feederServo.setPosition(SERVO_OPEN);
                    shootStartTime = currentTime;
                    shootStep++;
                }
                break;

            case 5:
                if (currentTime - shootStartTime > 0.2) {
                    feederServo.setPosition(SERVO_CLOSED);
                    shootStep = 0;
                }
                break;
        }
    }

    private void stopRobot() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        intakeMotor.setPower(0);
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }
}