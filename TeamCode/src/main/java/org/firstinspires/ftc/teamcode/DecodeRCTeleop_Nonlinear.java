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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import java.util.function.Supplier;

import android.util.Log;

@Configurable
@TeleOp(name = "Decode RC Nonlinear Tele0p")
public class DecodeRCTeleop_Nonlinear extends OpMode {
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


    private Sorter sorter;
    private Intake intake;
    private Shooter shooter;

    private ElapsedTime timer;

    private String[] stack;

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

        intake = new Intake(hardwareMap);
        sorter = new Sorter(hardwareMap);
        shooter = new Shooter(hardwareMap);

        timer = new ElapsedTime();
        sorter.shift(0);
        sorter.door("Close");
        sorter.update();
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    private void drivebaseControl(){
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
    }

    // Delay:

    private void delay(double waitTime){
        double initialT = timer.milliseconds();
        while(timer.milliseconds() - initialT < waitTime){
            drivebaseControl();
        }
    }
    //Shot Execution
    private void shoot(){
        shooter.setPower(1, 1);
        intake.setPower(0);
        intake.update();
        //this.delay(3000);
        sorter.door("Open");
        shooter.openBlocker();
        sorter.update();
        this.delay(1000);

        for(int i = 0; i < 3; i++){
            sorter.shift(1);
            sorter.update();
            this.delay(1000);
            sorter.registerShot();
        }

        shooter.setPower(0, 0);
        sorter.door("Close");
        sorter.update();
        delay(750);
        sorter.shift(-3);
        sorter.update();
        delay(1000);

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

        //Set Intake Direction Based Upon Space:
        if(sorter.isFull()){
            intake.setPower(-1); //Set intake to reject
            shooter.setPower(1, 1); //Startup Shooter
        }
        else{
            intake.setPower(1);
        }
        delay(750);
        intake.update();

        sorter.detect(); //Detect potential artifact
        stack = sorter.getArtifactStack();
        if(!stack[2].equals("")){
            if(!sorter.isFull()){
                sorter.shift(2); //If ball intaked & not full, shift to shooting pos [0]
                sorter.update();
                delay(500);
            }
        }


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

        //Shoot Artifact (if possible)
        if(gamepad1.xWasPressed() || gamepad1.xWasReleased()){
            if(!sorter.isEmpty()){
                this.shoot();
            }
        }
//        if (gamepad1.xWasPressed()) {
//            if(sorter.hasGreen()){
//                stack = sorter.getArtifactStack();
//                if(stack[0].equals("Ball")){
//                    this.shoot();
//                }
//                else if(stack[1].equals("Ball")){
//                    sorter.shift(1);
//                    sorter.update();
//                    this.shoot();
//                }
//                else{
//                    sorter.shift(2);
//                    sorter.update();
//                    this.shoot();
//                }
//            }
//            else{
//                this.shoot();
//            }
//        }

//        //Shoot Purple Artifact (if possible)
//        if (gamepad1.yWasPressed()) {
//            if(sorter.hasGreen()){
//                stack = sorter.getArtifactStack();
//                if(stack[0].equals("Purple")){
//                    this.shoot();
//                }
//                else if(stack[1].equals("Purple")){
//                    sorter.shift(1);
//                    sorter.update();
//                    this.shoot();
//                }
//                else{
//                    sorter.shift(2);
//                    sorter.update();
//                    this.shoot();
//                }
//            }
//            else{
//                this.shoot();
//            }
//        }

        if(gamepad1.bWasPressed()){
            intake.setPower(1);
            intake.update();
        }


          /* Telemetry Outputs of our Follower */

//        telemetryM.debug("position", follower.getPose());
//        telemetryM.debug("velocity", follower.getVelocity());
//        telemetryM.debug("automatedDrive", automatedDrive);
//        telemetryM.debug("ballDetected",ballDetected);
//        telemetry.addData("Position", follower.getPose());
        stack = sorter.getArtifactStack();
        telemetry.addData("Artifacts: ", '{' + stack[0] + " | " + stack[1] + " | " + stack[2] + '}');
        telemetry.addData("SorterFull: ", sorter.isFull());

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