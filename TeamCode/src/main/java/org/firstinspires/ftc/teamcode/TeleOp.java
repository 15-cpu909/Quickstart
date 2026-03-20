package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
public class TeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Drive motors
    private DcMotor frontLeft, backLeft, frontRight, backRight;

    // NEW: Mechanisms
    private DcMotor intakeMotor;      // Core Hex
    private DcMotor shooterLeft;      // UltraPlanetary
    private DcMotor shooterRight;     // UltraPlanetary
    private Servo feederServo;

    // Servo positions
    private final double SERVO_CLOSED = 0.0;
    private final double SERVO_OPEN = 0.5;

    @Override
    public void runOpMode() {

        // Drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Mechanisms
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
        feederServo = hardwareMap.get(Servo.class, "feederServo");

        // Motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        shooterRight.setDirection(DcMotor.Direction.REVERSE); // IMPORTANT for wheels spinning same direction

        // Set servo default
        feederServo.setPosition(SERVO_CLOSED);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // ================= DRIVE =================
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // ================= INTAKE (LEFT TRIGGER) =================
            if (gamepad1.left_trigger > 0.1) {
                intakeMotor.setPower(0.8); // ~100 RPM
            } else {
                intakeMotor.setPower(0);
            }

            // ================= SHOOTER (RIGHT TRIGGER) =================
            if (gamepad1.right_trigger > 0.1) {
                shooterLeft.setPower(0.7);
                shooterRight.setPower(0.7);
            } else {
                shooterLeft.setPower(0);
                shooterRight.setPower(0);
            }

            // ================= SERVO FEED (LEFT BUMPER) =================
            if (gamepad1.left_bumper) {
                feederServo.setPosition(SERVO_OPEN);
                sleep(200); // quick nudge
                feederServo.setPosition(SERVO_CLOSED);
            }

            // ================= TELEMETRY =================
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Intake", intakeMotor.getPower());
            telemetry.addData("Shooter", "L: %.2f R: %.2f", shooterLeft.getPower(), shooterRight.getPower());
            telemetry.update();
        }
    }
}
