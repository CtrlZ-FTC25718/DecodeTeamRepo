package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.3)
            .forwardZeroPowerAcceleration(-38.5936)
            .lateralZeroPowerAcceleration(-81.1515)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.0007)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0.0, 0.0035, 0.05))
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.075, 0))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.005, 0, 0.00001, 0.6, 0)
            )
            .secondaryTranslationalPIDFCoefficients(
                    new PIDFCoefficients(0.1, 0, 0.005, 0)
            )
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.06, 0))
            .secondaryDrivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.1, 0, 0.00001, 0.6, 0)
            );

    public static DriveEncoderConstants robotConstants = new DriveEncoderConstants()
            .robotWidth(14.2)
            .robotLength(12.5984);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD)

//            .leftFrontMotorDirection(DcMotorEx.Direction.FORWARD)
//            .leftRearMotorDirection(DcMotorEx.Direction.FORWARD)
//            .rightFrontMotorDirection(DcMotorEx.Direction.REVERSE)
//            .rightRearMotorDirection(DcMotorEx.Direction.REVERSE)

            .xVelocity(48.3037) //81.07303
            .yVelocity(67.4132) //68.58403
            .useBrakeModeInTeleOp(true)
            .useVoltageCompensation(true);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(4) // was 2
            .strafePodX(25.4) // was 21.5
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .yawScalar(1.0)
            .encoderResolution(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            50,
            3.5,
            9
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .driveEncoderLocalizer(robotConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}

