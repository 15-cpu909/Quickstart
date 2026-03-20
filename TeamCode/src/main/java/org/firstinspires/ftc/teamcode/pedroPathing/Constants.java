package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants();

    /**
     * Mecanum drivetrain configuration
     */
    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftBack")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")

            // Motor directions (adjust if needed after testing)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    /**
     * Drive motor encoder localizer (NO dead wheels)
     */
    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .robotLength(14)     // inches (wheelbase front-back)
            .robotWidth(16.3)    // inches (wheelbase left-right)

            // ⚠️ TEMP: placeholder — MUST tune later
            // Replace with real values after using ForwardTuner
            .forwardTicksToInches(1.0)

            // Motor names (must match configuration exactly)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")

            // Encoder directions (you WILL likely need to tune these)
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD);

    /**
     * Path constraints (start conservative for motor encoder localization)
     */
    public static PathConstraints pathConstraints =
            new PathConstraints(
                    0.6,   // max power (reduced for accuracy)
                    40,    // max velocity
                    30,    // acceleration
                    2      // angular velocity
            );

    /**
     * Builds the follower
     */
    public static Follower createFollower(HardwareMap hardwareMap) {

        return new FollowerBuilder(followerConstants, hardwareMap)

                .mecanumDrivetrain(driveConstants)

                .driveEncoderLocalizer(localizerConstants)

                .pathConstraints(pathConstraints)

                .build();
    }
}
