package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "LeftFrontDrive";
        FollowerConstants.leftRearMotorName = "LeftRearDrive";
        FollowerConstants.rightFrontMotorName = "RightFrontDrive";
        FollowerConstants.rightRearMotorName = "RightRearDrive";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE; // was reverse
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE; // was reverse
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD; // was forward
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD; // was forward

        FollowerConstants.mass = 6.1;

        FollowerConstants.xMovement = 81.07303;
        FollowerConstants.yMovement = 68.58403;

        FollowerConstants.forwardZeroPowerAcceleration = -70.3007;
        FollowerConstants.lateralZeroPowerAcceleration = -93.9529;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1125,0.01,0.001,0);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.0875,0,0.02,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1.05,0,0.075,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.06,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01,0,0.00175,0.6,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.05,0,0.01,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 10.5; //was 36
        FollowerConstants.centripetalScaling = 0.0006;

        FollowerConstants.pathEndTimeoutConstraint = 50;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.01; // was 0.1
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        //Modified by Ragav Iyer: Function --> Braking/Voltage Compensation
        FollowerConstants.useBrakeModeInTeleOp = true;
        FollowerConstants.useVoltageCompensationInAuto = true;
        FollowerConstants.useVoltageCompensationInTeleOp = true;

    }
}
