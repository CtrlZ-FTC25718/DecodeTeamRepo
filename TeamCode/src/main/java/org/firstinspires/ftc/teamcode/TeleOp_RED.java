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
@TeleOp(name = "TeleOp: RED")
public class TeleOp_RED extends OpMode {
private Follower follower;
    public Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> farShotPathChain, closeShotPathChain, customShotPathChain, endgameChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25;
    private int shotCount;

    private Sorter sorter;
    private Intake intake;
    private Shooter shooter;

    private ElapsedTime timer;
    private double[] delayTimer;
    private String[] stack;
    private double artifactCountBefore, artifactCountAfter;
    private boolean shootArtifactAtHighSpeed, shootArtifactAtLowSpeed, shootArtifactAtCustomSpeed;
    private double[] customParameters = {0.0, 0.0};

    private String teamColor; // Set to Red or Blue

    //private limelightDetector visionDetector;


    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        // set Team Color to Red or Blue - Should match with the teleOp Name
        teamColor = "Red";
        // Set Starting Pose
        startingPose = new Pose(72,8.5, Math.toRadians(90));; //See ExampleAuto to understand how to use this


        follower = Constants.createFollower(hardwareMap);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        
        
        farShotPathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(84, 26))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(67), .8))
                .build();

        closeShotPathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(90, 90))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), .8))
                .build();

        endgameChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(38, 38))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), .8))
                .build();

        intake = new Intake(hardwareMap);
        sorter = new Sorter(hardwareMap);
        shooter = new Shooter(hardwareMap);

        timer = new ElapsedTime();


        sorter.initializeIndicatorColor();


        shootArtifactAtHighSpeed = false;
        shootArtifactAtLowSpeed = false;
        shootArtifactAtCustomSpeed = false;
        shotCount = 0;
        artifactCountBefore = 0;
        artifactCountAfter = 0;

        // Use shotTimer to delay for various reasons\
        // Index 0 used for delaying intake reversal when sorter is full
        // Index 1 used for delaying sorter rotation for intake
        // Index 2 used for delaying ball detection
        // Index 3 used for delaying for door opening to shoot artifact
        // Index 4 used for delaying sorter rotation for shooting
        delayTimer = new double[5]; // create 5 delayTimers that can used for various (non-blocking) delays; sets to 0.0s


        //Limelight Camera
        //visionDetector = new limelightDetector(hardwareMap);
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    // General timerExpiration Check function
    private boolean timerExpired(int timerIndex , double waitTime){
        return (timer.milliseconds() - delayTimer[timerIndex] > waitTime);
    }

    private void shootArtifact (){
        if (timerExpired(3,1000)){
            //Log.d("Shooter1", "Sorter Door Timer Expired");

            if(!sorter.isEmpty()){
//                Log.d("Shooter2", "Sorter Not empty, waiting for timer 4 to expire");
                if  (timerExpired(4, 750)) {
//                    Log.d("Shooter3", "Sorter Timer Expired");

                    if (sorter.hasDoorOpened()) {
                        //Log.d("Shooter4", "Sorter Door Is Open");

                        // Long shot
                        if (shootArtifactAtHighSpeed) {

                            //Log.d("Shooter5", "Long shot active");
                            if (shooter.isAtHighVel()) {
//                                Log.d("Shooter5p1", "Shooter reached high Vel");

                                if (!sorter.isEmpty() && shotCount > 1) { //First artifact is already in the shooter when the door opened
                                    sorter.shift(1);
                                    sorter.update();
                                    //Log.d("ShooterShift", "Sorter Has Shifted");
                                }
                                shooter.openBlocker();

                                stack = sorter.getArtifactStack();
//                                Log.d("ShootingStackBefore", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);
                                sorter.registerShot();
                                stack = sorter.getArtifactStack();
//                                Log.d("ShootingStackAfter", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);

                                shotCount++;

                                stack = sorter.getArtifactStack();
                                //Log.d("ShootingStackAfterShift", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);


                                delayTimer[4] = timer.milliseconds(); // Rest for delaying sorter next round
                                //Log.d("Shooter6", "Shot Registered at high speed: " + shotCount);
                            }
                            else {
                                //Log.d("Shooter7", "Shooter isAtHighVel is false");
                            }
                        }

                        // Close shot
                        else if (shootArtifactAtLowSpeed) {

                            //Log.d("Shooter5p2", "Close shot active");
                            if (shooter.isAtLowVel()) {
                                //Log.d("Shooter5p2", "Shooter reached low Vel");

                                if (!sorter.isEmpty() && shotCount > 1) { //First artifact is already in the shooter when the door opened
                                    sorter.shift(1);
                                    sorter.update();
                                    //Log.d("ShooterShift", "Sorter Has Shifted");
                                }
                                shooter.openBlocker();

                                stack = sorter.getArtifactStack();
                                //Log.d("ShootingStackBefore", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);
                                sorter.registerShot();
                                stack = sorter.getArtifactStack();
                                //Log.d("ShootingStackAfter", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);

                                shotCount++;

                                stack = sorter.getArtifactStack();
                                //Log.d("ShootingStackAfterShift", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);


                                delayTimer[4] = timer.milliseconds(); // Rest for delaying sorter next round
                                //Log.d("Shooter6", "Shot Registered at high speed: " + shotCount);
                            }
                            else {
                                //Log.d("Shooter7", "Shooter isAtLowVel is false");
                            }
                        }

                        // Long shot
                        if (shootArtifactAtCustomSpeed) {

                            //Log.d("Shooter5", "Long shot active");

                            if (shooter.isAtCustomVel()) {
//                                Log.d("Shooter5p1", "Shooter reached custom vel");

                                if (!sorter.isEmpty() && shotCount > 1) { //First artifact is already in the shooter when the door opened
                                    sorter.shift(1);
                                    sorter.update();
                                    //Log.d("ShooterShift", "Sorter Has Shifted");
                                }
                                shooter.openBlocker();

                                stack = sorter.getArtifactStack();
//                                Log.d("ShootingStackBefore", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);
                                sorter.registerShot();
                                stack = sorter.getArtifactStack();
//                                Log.d("ShootingStackAfter", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);

                                shotCount++;

                                stack = sorter.getArtifactStack();
                                //Log.d("ShootingStackAfterShift", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);


                                delayTimer[4] = timer.milliseconds(); // Rest for delaying sorter next round
                                //Log.d("Shooter6", "Shot Registered at custom speed: " + shotCount);
                            }
                            else {
                                //Log.d("Shooter7", "Shooter isAtCustomVel is false");
                            }
                        }
                        else {
                            //Log.d("Shooter11", "shootArtifactAtHighSpeed && shootArtifactAtLowSpeed & shootArtifactAtCustomVel are false");
                        }
                    }


                    else{
                        //Log.d("Shooter12", "Door is not open");
                    }
                }


                else {
                    //Log.d("Shooter13", "Waiting for delayTimer[4] to expire");
                }
            }
            else {
                if (timerExpired(4, 1000)){
                    // Sorter is empty; But will execute until shootArtifactAtHighSpeed is false
                    // Execute end actions after shooting artifacts
//                    Log.d("Shooter14", "Sorter Empty End Shooting and Reset Timers");
                    sorter.door("Close");
                    sorter.reset();
                    sorter.update();

                    shooter.setVelocity("Idle");

                    // Done shooting set shooting states to false
                    if (shootArtifactAtHighSpeed) { shootArtifactAtHighSpeed = false;}
                    if (shootArtifactAtLowSpeed){ shootArtifactAtLowSpeed = false;}
                    if (shootArtifactAtCustomSpeed){ shootArtifactAtCustomSpeed = false;};

                    shotCount = 0; // Reset Shot counter

                    // Rest all delay timers
                    delayTimer[0] = 0; // Reset Intake Delay Timer
                    delayTimer[1] = 0; // Sorter rotate Delay Timer (for intake)
                    delayTimer[2] = 0; // Ball detect Delay Timer
                    delayTimer[3] = 0; // Reset Shooter Door Delay Timer
                    delayTimer[4] = 0; // Sorter rotate Delay Timer (for shooting)
                }

            }
        }
    }

    // For automatic intake management
    public void manageIntake(){
        //Set Intake Direction Based Upon Space:
        if(sorter.isFull()){
            if (timerExpired(0,5000)) {
                intake.setIntakeState(false);
                intake.update();
            }
        }
    }

    public void manageSorter(){
        if(sorter.isFull() && delayTimer[0] == 0){
            delayTimer[0] = timer.milliseconds(); // Start a new timer to stop the intake and reject artifacts
            // startShooter for initial spin up (will reduce velocity Hold time to overcome inertia
            shooter.setVelocity("Low");
        }

        if((timerExpired(2, 250) || delayTimer[2] == 0) && !shootArtifactAtHighSpeed && !shootArtifactAtLowSpeed){
            stack = sorter.getArtifactStack();
            //Log.d("ShootingStackBeforeDetect", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);

            artifactCountBefore = sorter.getArtifactCount();
            sorter.detect(); //Detect potential artifact
            stack = sorter.getArtifactStack();
            artifactCountAfter = sorter.getArtifactCount();

            //Log.d("Shifting1", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);

            stack = sorter.getArtifactStack();
            //Log.d("ShootingStackAfterDetect", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);


            delayTimer[2] = 0;
        }

        if(!stack[2].equals("") && !shootArtifactAtHighSpeed && !shootArtifactAtLowSpeed && !shootArtifactAtCustomSpeed){
            if(!sorter.isFull()){

                if(delayTimer[1] == 0){
                    delayTimer[1] = timer.milliseconds(); // Start a new timer to wait to rotate the sorter
                }
                if(timerExpired(1, 1)){
                    sorter.shift(1); //If ball has been taken in & sorter once (used to sort twice)
                    sorter.update();
                    delayTimer[1] = 0;
                    delayTimer[2] = timer.milliseconds();
                    //Log.d("Shifting2", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);

                }
            }
        }

    }

    public void toggleIntake(){
        intake.setIntakeState(!intake.getIntakeState());
        intake.update();
        // Handle Shooter Spin Up
        if(intake.getIntakeState()){
            shooter.setVelocity("Idle");
        }
        else {
            shooter.setVelocity("Low");
        }
    }
    public void toggleDirection(){
        intake.setDirection(-1 * intake.getDirection());
        intake.update();
    }
    public void driveControl(){
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */

            double rsy = gamepad1.right_stick_y;
            double rsx = gamepad1.right_stick_x;
            double lsx = gamepad1.left_stick_x;

            //This is how it looks with slowMode on
            // Scale normal driving as a quadratic X^2
            if (!slowMode) {

                if (rsy > 0){ rsy = -1.5*Math.pow(rsy,2);}
                else {rsy = 1.5*Math.pow(rsy,2);}

                if (rsx > 0){ rsx = -1.5*Math.pow(rsx,2);}
                else {rsx = 1.5*Math.pow(rsx,2);}

                if (lsx > 0){ lsx = -1.5*Math.pow(lsx,2);}
                else {lsx = 1.5*Math.pow(lsx,2);}

                follower.setTeleOpDrive(
                        rsy,
                        rsx,
                        lsx,
                        true // Robot Centric
                );
            }
            else follower.setTeleOpDrive(
                    -rsy * slowModeMultiplier,
                    -rsx * slowModeMultiplier,
                    -lsx * slowModeMultiplier,
                    true // Robot Centric
            );
        }
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();

//        sorter.setPosition(0.188);
//        sorter.setArtifactStack(new String[]{"", "", ""}); //Set Sorter State for Preloads
        sorter.door("Close");
        sorter.update();

        shooter.setVelocity("Idle");
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        //Call this once per loop
        follower.update();
        telemetryM.update();
        telemetry.update();
        shooter.computeEnergy();

        // Drive Control
        driveControl();
        
        // Manage number of balls in the sorter and determine when to rotate ball to shooter position
        manageSorter();
        
        //Intake Control
        manageIntake();

        // Set RGB Indicator Color by ball count
        sorter.setIndicatorLightColor();
        

        // Shoot Artifact without holding the thread if triggered
        if (shootArtifactAtHighSpeed || shootArtifactAtLowSpeed || shootArtifactAtCustomSpeed){
            shootArtifact();
        }

        // Handle/Respond to button clicks

        // Go to Long Shot artifact shooting position
        if (gamepad1.a || gamepad1.aWasPressed()) {
            follower.followPath(farShotPathChain.get(),true);
            // Wait till the follower is done. Allows to process pose faster. But need to update follower while waiting
            while(follower.isBusy()){follower.update();}
            automatedDrive = true;
        }

        if (gamepad1.b || gamepad1.bWasPressed()) {
            follower.followPath(closeShotPathChain.get(),true);
            automatedDrive = true;
        }
        //Stop auto
        //Stop automated following if the follower is done
        if (automatedDrive && ((gamepad1.left_trigger > 0) || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        if (gamepad1.right_trigger > 0){
            intake.setDirection(-1);
            toggleIntake();
        }

        //Slow Mode
        if (gamepad1.leftBumperWasPressed()) {
            slowMode = !slowMode;
        }

        // Toggle Intake
        if (gamepad1.rightBumperWasPressed()) {
            intake.setDirection(1);
            intake.setIntakeState(!intake.getIntakeState());
            intake.update();
            // Handle Shooter Spin Up
            if(intake.getIntakeState()){
                shooter.setVelocity("Idle");
            }
            else {
                shooter.setVelocity("Low");
            }

        }

        // Far shot
        if(gamepad1.yWasPressed() || gamepad1.yWasReleased()){
            //Shoot Artifacts (everything that is in the stack)
            if(!sorter.isEmpty()){
                shootArtifactAtHighSpeed = true;
                shooter.velocityHold("High", .1); // initial spinup
                shotCount = 0;
                sorter.door("Open");
                sorter.update();
                sorter.wiggleUp();
                intake.setIntakeState(false);
                intake.update();
                shooter.closeBlocker(); // do not shoot until velocity is reached
                shooter.setVelocity("High");

                delayTimer[3] = timer.milliseconds(); // Set Door Delay Timer for shooting
                delayTimer[4] = timer.milliseconds(); // Set Sorter Delay Timer for Shooting
            }
        }

        // Close shot
        if(gamepad1.xWasPressed() || gamepad1.xWasReleased()){
            //Shoot Artifacts (everything that is in the stack)
            if(!sorter.isEmpty()){
                shootArtifactAtLowSpeed = true;
                shooter.velocityHold("Low", .1); // initial spin up
                shotCount = 0;
                sorter.door("Open");
                sorter.update();
                sorter.wiggleUp();
                intake.setIntakeState(false);
                intake.update();

                shooter.closeBlocker(); // do not shoot until velocity is reached
                shooter.setVelocity("Low");

                delayTimer[3] = timer.milliseconds(); // Set Door Delay Timer for shooting
                delayTimer[4] = timer.milliseconds(); // Set Sorter Delay Timer for Shooting
            }
        }

        // Custom Shot
        if(gamepad1.guideWasPressed() || gamepad1.guideWasReleased()){
            //Shoot Artifacts (everything that is in the stack)
            if(!sorter.isEmpty()){
                shootArtifactAtCustomSpeed = true;

                if(follower.getPose().getY() <= 48){
                    // Far shot; Use default position
                    follower.followPath(farShotPathChain.get(),true);
                    automatedDrive = true;

                    shooter.velocityHold("High", .1); // inital spinup
                    shotCount = 0;
                    sorter.door("Open");
                    sorter.update();
                    sorter.wiggleUp();
                    intake.setIntakeState(false);
                    intake.update();
                    shooter.closeBlocker(); // do not shoot until velocity is reached
                    shooter.setVelocity("High");
                }
                else{
                    // For custom shot (close) calculate ComputeCustomShotParameters
                    customParameters = shooter.computeCustomShotParameters(follower.getPose().getX(), follower.getPose().getY(),teamColor);
                    shooter.updateCustomVelocity(customParameters[0], customParameters[0]); // use same values for front and back

                    // Close shot; offset
                    // Turn robot to calculated heading and turn

                    customShotPathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                            .addPath(new Path(new BezierLine(follower::getPose, new Pose(customParameters[2], customParameters[3], customParameters[1])))) // No change in x,y.
                            .setConstantHeadingInterpolation(Math.toRadians(customParameters[1]))
                            .build();
                    follower.followPath(customShotPathChain.get(),true);
                    automatedDrive = true;

                    shooter.velocityHold("Custom", .1); // inital spinup
                    shotCount = 0;
                    sorter.door("Open");
                    sorter.update();
                    sorter.wiggleUp();
                    intake.setIntakeState(false);
                    intake.update();
                    shooter.closeBlocker(); // do not shoot until velocity is reached
                    shooter.setVelocity("Custom");
                }

                delayTimer[3] = timer.milliseconds(); // Set Door Delay Timer for shooting
                delayTimer[4] = timer.milliseconds(); // Set Sorter Delay Timer for Shooting
            }
        }
        if(gamepad2.dpadLeftWasPressed()){
            sorter.shift(-1);
            sorter.update();
        }

        if(gamepad2.dpadRightWasPressed()){
            sorter.shift(1);
            sorter.update();
        }

        if(gamepad2.dpadUpWasPressed()){
            sorter.wiggleUp();
        }

        if(gamepad2.dpadDownWasPressed()){
            sorter.reset();
        }

        if(gamepad2.bWasPressed() || gamepad2.b){
            shooter.openBlocker();
        }

        if(gamepad2.aWasPressed() || gamepad2.a){
            shooter.closeBlocker();
        }

        if(gamepad2.xWasPressed() || gamepad2.x){
            sorter.door("Close");
            sorter.update();
        }

        if(gamepad2.yWasPressed() || gamepad2.y){
            sorter.door("Open");
            sorter.update();
        }

        if(gamepad2.guideWasPressed() || gamepad2.guideWasReleased()){
            follower.setPose(startingPose);
            follower.update();

        }

        if(gamepad2.rightBumperWasPressed()){
            toggleDirection();

        }

        if(gamepad2.leftBumperWasPressed()){
            intake.releaseJam();
        }

        if(gamepad2.left_trigger != 0){
            sorter.setArtifactStack(new String[]{"Ball", "Ball", "Ball"}); //Set Sorter State for Preloads
            sorter.update();
        }

        if(gamepad2.right_trigger != 0){
            follower.followPath(endgameChain.get(), 1, false);
            automatedDrive = true;
        }



        /* if(gamepad2.rightBumperWasPressed()){
            if(intake.getPower() == 0){
                intake.setPower(-1);
            }
            else{
                intake.setPower(0);
            }
        }
        if(gamepad2.leftBumperWasPressed()){
            if(intake.getPower() == 0){
                intake.setPower(1);
            }
            else{
                intake.setPower(0);
            }
        } */

        stack = sorter.getArtifactStack();
//        telemetry.addData("IsAutomatedDriveMode: ", "" + automatedDrive);
//        telemetry.addData("SorterFull: ", sorter.isFull());

        telemetry.addData("ShooterFrontVel t/s: ", shooter.getShooterFrontVel());
        telemetry.addData("ShooterBackVel t/s: ", shooter.getShooterBackVel());

//        telemetry.addData("SorterPos: ", sorter.getPosition());

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        //double[] visionPosEst =  visionDetector.visualLocalization(new double[]{126, 134.5, 135}, 24, 1);
//
//        telemetry.addData("CameraX", visionPosEst[0] + "");
//        telemetry.addData("CameraY", visionPosEst[1] + "");
//        telemetry.addData("CameraTheta", visionPosEst[2] + "");


//        telemetry.addLine("--------CUSTOM SHOT PARAMS-----------");
//        telemetry.addData("Custom Shooter Vel t/s", customParameters[0]);
//        telemetry.addData("Custom Shooting Heading in Degrees", customParameters[1]);


    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}