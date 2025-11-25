package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;
import org.firstinspires.ftc.teamcode.MappedActuators;
/*
 * Team CTRL+Z # 25718
 * @version 2.0, 12/30/2024
*/

import java.util.function.Supplier;
@Configurable
@TeleOp(name = "Decode Robot Centric Tele0p")
public class DecodeRCTeleop extends OpMode {
private Follower follower;
    public Pose startingPose  = new Pose(0,0,0);; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.2;
    public MappedActuators robotActuators;


    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
        robotActuators = new MappedActuators(hardwareMap);
        robotActuators.resetSorter(); // Reset Sorter position and state
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */
        //Call this once per loop
        follower.update();
        telemetryM.update();
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.right_stick_y,
                    -gamepad1.right_stick_x,
                    -gamepad1.left_stick_x,
                    true // Robot Centric
            );
                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.right_stick_y * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }
        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }
        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }
        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }
        //Optional way to change slow mode strength
        if (gamepad1.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

        // Intake - Intake action
        if (gamepad1.dpadDownWasPressed()) {
            // Intake artifact
            robotActuators.spinIntake("intake");
        }
        // Intake - Reject action
        if (gamepad1.dpadUpWasPressed()){
            // Reject artifact
            robotActuators.spinIntake("reject");
        }
        if ((gamepad1.dpadLeftWasPressed()) || (gamepad1.dpadRightWasPressed())) {
            // Stop intake
            robotActuators.spinIntake("stop");
        }

        if(gamepad2.yWasPressed()){
            // Start shooter
            robotActuators.spinShooter("start");
        }

        if(gamepad2.aWasPressed()){
            // Stop shooter
            robotActuators.spinShooter("stop");
        }

        if(gamepad2.dpadRightWasPressed()){
            // Sort once Forward
            robotActuators.sortArtifactForward();
        }

        if(gamepad2.dpadLeftWasPressed()){
            // Sort once Backward
            robotActuators.sortArtifactBackward();
        }
        if ((gamepad2.dpadUpWasPressed()) || (gamepad2.dpadDownWasPressed())) {
            // Stop intake
            robotActuators.resetSorter();
        }

        if (gamepad2.rightBumperWasPressed()){
            robotActuators.doorOpen();
        }
        if (gamepad2.leftBumperWasPressed()){
            robotActuators.doorClosed();
        }
        if (gamepad2.xWasPressed()){
            robotActuators.unblockShooter();
        }
        if (gamepad2.bWasPressed()){
            robotActuators.blockShooter();
        }

        /* Telemetry Outputs of our Follower */

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

//        telemetry.addData("X", follower.getPose().getX());
//        telemetry.addData("Y", follower.getPose().getY());
//        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
//        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}