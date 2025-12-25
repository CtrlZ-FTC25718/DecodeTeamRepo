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
            .forwardZeroPowerAcceleration(-31.308) // -34.38, -34.19, -29.69, -27.69, -30.59
            .lateralZeroPowerAcceleration(-68.952) //-69.29 -68.44, 68.47, -68.84, -69.72
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.0007)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1.25, 0, 0.005, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0075, 0, 0.0005, 0.6, 0))
            /*.useSecondaryTranslationalPIDF(false)
            .useSecondaryDrivePIDF(false)
            .useSecondaryHeadingPIDF(false)
            .centripetalScaling(.0007)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.9, 0.1, 0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0.1, 0, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.9, 0.25, 0, 0.6, 0))*/
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.02, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.75, 0, 0.003, 0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(100, 0, 0.001, 0.6, 0))
            .headingPIDFSwitch(0.07)
            .translationalPIDFSwitch(5);


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

            .xVelocity(62.922) //63.3, 62.9, 63.3, 62.09, 63.02
            .yVelocity(50.732) //50.73, 50.4, 50.91, 50.66, 50.97
            .useVoltageCompensation(true)
            .useBrakeModeInTeleOp(true);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5)
            .strafePodX(31.75)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .yawScalar(1.0)
            .encoderResolution(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.85,
            250,
            1.75,
            15
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

