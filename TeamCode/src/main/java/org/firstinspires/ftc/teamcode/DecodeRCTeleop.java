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
/*
 * Team CTRL+Z # 25718
 * @version 2.0, 12/30/2024
*/

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "Decode Robot Centric Tele0p")
public class DecodeRCTeleop extends OpMode {
private Follower follower;
    public Pose startingPose  = new Pose(0,0, Math.toRadians(90));; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25;
    public MappedActuators robotActuators;

    private boolean ballDetected = false;
    private boolean prevBallDetected = false;
    private double RGBColor = 0;

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
        robotActuators.blockShooter();
        robotActuators.closeDoor();
        robotActuators.setRGBIndicatorTo(0.67);
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

        //robotActuators.setRGBIndicatorByIntakeBallCount();
        ballDetected = robotActuators.detectBall();
        if (!prevBallDetected && ballDetected && robotActuators.getIntakeBallCount() == 0) {
            robotActuators.delay("high");
            robotActuators.sorterGoToState(1, false);
            robotActuators.incrementIntakeBallCount(); // set to 1
            //RGBColor = 0.25;
            //robotActuators.setRGBIndicatorTo(RGBColor);
        }
        else if (!prevBallDetected && ballDetected && robotActuators.getIntakeBallCount() == 1) {
            robotActuators.sorterGoToState(2, false);
            robotActuators.incrementIntakeBallCount(); // set to 2
            //RGBColor = 0.5;
            //robotActuators.setRGBIndicatorTo(RGBColor);
        }
        else if (!prevBallDetected && ballDetected && robotActuators.getIntakeBallCount() == 2) {
            robotActuators.sorterGoToState(3, false);
            robotActuators.incrementIntakeBallCount(); // set to 3
            //RGBColor = 0.555;
            //robotActuators.setRGBIndicatorTo(RGBColor);
        }
        else if(robotActuators.getIntakeBallCount() == 3){
            robotActuators.spinIntake("stop");
            robotActuators.spinShooter("start", "high");
        }
        prevBallDetected = ballDetected;

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
        //shoot at highPower
        if (gamepad1.xWasPressed()) {
            robotActuators.shootBall("high");
        }
        //shoot at lowPower
        if (gamepad1.yWasPressed()) {
            robotActuators.shootBall("low");
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

        if(gamepad2.yWasPressed()) {
            // Start shooter
            robotActuators.spinShooter("start", "high");
            if (!robotActuators.getDoorOpenState()) {
                robotActuators.unblockShooter();
            }
            else {
                robotActuators.blockShooter();
            }
        }
        if(gamepad2.aWasPressed()){
            // Stop shooter
            robotActuators.spinShooter("stop", "low");
            if (!robotActuators.getDoorOpenState()) {
                robotActuators.unblockShooter();
            }
            else {
                robotActuators.blockShooter();
            }
        }

        if(gamepad2.dpadRightWasPressed()){
            // Sort once Forward
            robotActuators.sorterGoToState(robotActuators.getSorterState()+1, false);
            //robotActuators.setSorterPosition(robotActuators.getSorterPosition() + .005);
        }

        if(gamepad2.dpadLeftWasPressed()){
            // Sort once Backward
            robotActuators.sorterGoToState(robotActuators.getSorterState()-1, false);
            //robotActuators.setSorterPosition(robotActuators.getSorterPosition() - .005);
        }
        if ((gamepad2.dpadUpWasPressed()) || (gamepad2.dpadDownWasPressed())) {
            // Stop intake
            robotActuators.resetSorter();
        }

        if (gamepad2.rightBumperWasPressed() && !robotActuators.getDoorOpenState()){
            robotActuators.openDoor();
        }
        if (gamepad2.leftBumperWasPressed() && robotActuators.getDoorOpenState()){
            robotActuators.closeDoor();
        }
        if (gamepad2.xWasPressed()){
            robotActuators.unblockShooter();
        }
        if (gamepad2.xWasReleased()){
            robotActuators.blockShooter();
        }


        /* Telemetry Outputs of our Follower */

//        telemetryM.debug("position", follower.getPose());
//        telemetryM.debug("velocity", follower.getVelocity());
//        telemetryM.debug("automatedDrive", automatedDrive);
//        telemetryM.debug("ballDetected",ballDetected);
//        telemetry.addData("Position", follower.getPose());
        telemetry.addData("ballDetected", ballDetected);
        telemetry.addData("PrevBallDetected", prevBallDetected);
        telemetry.addData("intakeBallCount", robotActuators.getIntakeBallCount());

        telemetry.addData("R: ",robotActuators.getSorterBottomColorSensorRValue());
        telemetry.addData("G: ",robotActuators.getSorterBottomColorSensorGValue());
        telemetry.addData("B: ",robotActuators.getSorterBottomColorSensorBValue());
        telemetry.addData("sorterState", robotActuators.getSorterState());
        telemetry.addData("sorterPosition", robotActuators.getSorterPosition());

        telemetry.addData("doorServoPosition", robotActuators.door.getPosition());
        telemetry.addData("DoorOpenState", robotActuators.getDoorOpenState());
        telemetry.addData("shooterBlockerPosition", robotActuators.getShooterBlockerPosition());

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
//        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}