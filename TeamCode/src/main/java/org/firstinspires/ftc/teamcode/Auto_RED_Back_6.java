package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import java.util.function.Supplier;

@Configurable
@Autonomous(name = "RED: Back - 6")
public class Auto_RED_Back_6 extends OpMode {
    private Follower follower;
    public Pose startingPose;
    private TelemetryManager telemetryM;
    private int shotCount;
    private boolean isShooting;

    private Sorter sorter;
    private Intake intake;
    private Shooter shooter;

    private ElapsedTime timer;
    private double[] delayTimer;
    private String[] stack;
    private boolean shootArtifactAtHighSpeed, shootArtifactAtLowSpeed, shootArtifactAtCustomSpeed;
    private double[] customParameters = {0.0, 0.0};

    private String teamColor; // Set to Red or Blue

    //Autonomous Variables
    private Supplier<PathChain> farShotPoint, closeShotPoint, firstCollectionChain_0, firstCollectionChain_1, secondCollectionChain_0, secondCollectionChain_1, endingChain, customShotPathChain;

    private int pathState;
    private boolean shotParametersComputed;
    private double pathChainTimer;


    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        // set Team Color to Red or Blue - Should match with the teleOp Name
        teamColor = "Red";
        // Set Starting Pose
        startingPose= new Pose(96,9, Math.toRadians(90));; //See ExampleAuto to understand how to use this
//        startingPose= new Pose(124,124, Math.toRadians(45));;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


        intake = new Intake(hardwareMap);
        sorter = new Sorter(hardwareMap);
        shooter = new Shooter(hardwareMap);

        timer = new ElapsedTime();
        sorter.reset(); //Reset sorter to SorterZeroPos
//        sorter.shift(0);
        sorter.door("Close");
        sorter.update();

        sorter.initializeIndicatorColor();


        shootArtifactAtHighSpeed = false;
        shootArtifactAtLowSpeed = false;
        shootArtifactAtCustomSpeed = false;
        isShooting = false;
        shotCount = 0;

        // Use shotTimer to delay for various reasons\
        // Index 0 used for delaying intake reversal when sorter is full
        // Index 1 used for delaying sorter rotation for intake
        // Index 2 used for delaying ball detection
        // Index 3 used for delaying for door opening to shoot artifact
        // Index 4 used for delaying sorter rotation for shooting
        // Index 5 used for follower wait
        delayTimer = new double[6]; // create 4 delayTimers that can used for various (non-blocking) delays; sets to 0.0s

        // Autonomous Initialization
        shotParametersComputed = false;
        shooter.closeBlocker();
        intake.resetSlapper();
        pathState = 0;
        sorter.setArtifactStack(new String[]{"Ball", "Ball", "Ball"}); //Set Sorter State for Preloads

        // Autonomous Path Construction
        farShotPoint = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(84, 26))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(70), .8))
                .build();

        firstCollectionChain_0 = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(96, 34))))
                .setHeadingInterpolation(HeadingInterpolator.constant(Math.toRadians(0)))
                .build();

        firstCollectionChain_1 = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(136, 34))))
                .setHeadingInterpolation(HeadingInterpolator.constant(Math.toRadians(0)))
                .build();

        secondCollectionChain_0 = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(96, 56))))
                .setHeadingInterpolation(HeadingInterpolator.constant(Math.toRadians(0)))
                .build();
        secondCollectionChain_1 = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(136, 56))))
                .setHeadingInterpolation(HeadingInterpolator.constant(Math.toRadians(0)))
                .build();

        endingChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(84, 38))))
                .setHeadingInterpolation(HeadingInterpolator.constant(Math.toRadians(55)))
                .build();

        closeShotPoint = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierCurve(follower::getPose, new Pose(90, 67))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), .8))
                .build();

//        firstCollectionChain = () -> follower.pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(follower::getPose, new Pose(96, 36))))
//                .addPath(new Path(new BezierLine(follower::getPose, new Pose(110, 36))))
//                .addPath(new Path(new BezierLine(follower::getPose, new Pose(115, 36))))
//                .addPath(new Path(new BezierLine(follower::getPose, new Pose(135, 36))))
//                .setHeadingInterpolation(HeadingInterpolator.constant(Math.toRadians(0)))
//                .build();
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
        Log.d("Shooter0","Inside ShootArtifact");
        isShooting = true;
        if (timerExpired(3,750)){
            Log.d("Shooter1", "Sorter Door Timer Expired");

            if(!sorter.isEmpty()){
                Log.d("Shooter2", "Sorter Not empty, waiting for timer 4 to expire");
                if  (timerExpired(4, 750)) {
                    Log.d("Shooter3", "Sorter Timer Expired");

                    if (sorter.hasDoorOpened()) {
                        Log.d("Shooter4", "Sorter Door Is Open");

                        // Long shot
                        if (shootArtifactAtHighSpeed) {

                            //Log.d("Shooter5", "Long shot active");
                            if (shooter.isAtHighVel()) {
//                                Log.d("Shooter5p1", "Shooter reached high Vel");

                                if (!sorter.isEmpty() && shotCount > 1) { //First artifact is already in the shooter when the door opened
                                    sorter.shift(1);
                                    sorter.update();
                                    sorter.wiggleUp(); // Added only for Auto
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

                            Log.d("Shooter5p2", "Close shot active");
                            if (shooter.isAtLowVel()) {
                                Log.d("Shooter5p2", "Shooter reached low Vel");

                                if (!sorter.isEmpty() && shotCount > 1) { //First artifact is already in the shooter when the door opened
                                    sorter.shift(1);
                                    sorter.update();
                                    Log.d("ShooterShift", "Sorter Has Shifted");
                                }
                                shooter.openBlocker();

                                stack = sorter.getArtifactStack();
                                Log.d("ShootingStackBefore", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);
                                sorter.registerShot();
                                stack = sorter.getArtifactStack();
                                Log.d("ShootingStackAfter", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);

                                shotCount++;

                                stack = sorter.getArtifactStack();
                                Log.d("ShootingStackAfterShift", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);


                                delayTimer[4] = timer.milliseconds(); // Rest for delaying sorter next round
                                Log.d("Shooter6", "Shot Registered at high speed: " + shotCount);
                            }
                            else {
                                Log.d("Shooter7", "Shooter isAtLowVel is false");
                            }
                        }

                        // Long shot
                        if (shootArtifactAtCustomSpeed) {

                            //Log.d("Shooter5", "Long shot active");

                            if (shooter.isAtCustomVel()) {
                                //Log.d("Shooter5p1", "Shooter reached custom vel");

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
                    if (shootArtifactAtCustomSpeed){ shootArtifactAtCustomSpeed = false;}
                    if (isShooting) { isShooting = false;}

                    shotCount = 0; // Reset Shot counter

//                    pathState++; //Move on to next autonomous step

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

        if((timerExpired(2, 500) || delayTimer[2] == 0) && !shootArtifactAtHighSpeed && !shootArtifactAtLowSpeed){
            stack = sorter.getArtifactStack();
            //Log.d("ShootingStackBeforeDetect", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);

            sorter.detect(); //Detect potential artifact
            stack = sorter.getArtifactStack();
            //Log.d("Shifting1", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);

            stack = sorter.getArtifactStack();
            //Log.d("ShootingStackAfterDetect", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);


            delayTimer[2] = 0;
        }

        if(!stack[2].equals("") && !shootArtifactAtHighSpeed && !shootArtifactAtLowSpeed && !shootArtifactAtCustomSpeed){
            if(!sorter.isFull()){

                if(delayTimer[1] == 0){
                    delayTimer[1] = timer.milliseconds(); // Start a new timer to wait to rotate the sorter
                    toggleIntake();
                }
                if(timerExpired(1, 1)){
                    toggleIntake();
                    sorter.shift(1); //If ball has been taken in & sorter once, used to sort twice
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

    public void customShot(){
        //Shoot Artifacts (everything that is in the stack)
        if(!sorter.isEmpty() && !shotParametersComputed){
            shootArtifactAtCustomSpeed = true;
            shotParametersComputed = true;

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

            shooter.velocityHold("Custom", .1); // inital spinup
            shotCount = 0;
            sorter.door("Open");
            sorter.update();
            sorter.wiggleUp();
//            intake.setIntakeState(false);
//            intake.update();
            shooter.closeBlocker(); // do not shoot until velocity is reached
            shooter.setVelocity("Custom");


            delayTimer[3] = timer.milliseconds(); // Set Door Delay Timer for shooting
            delayTimer[4] = timer.milliseconds(); // Set Sorter Delay Timer for Shooting
        }
    }

    public void farShot(){
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

    public void closeShot(){
        //Shoot Artifacts (everything that is in the stack)
        if(!sorter.isEmpty()){
            shootArtifactAtLowSpeed = true;
            shooter.velocityHold("Low", .1); // initial spin up
            shotCount = 0;
            sorter.door("Open");
            sorter.update();
            sorter.wiggleUp();
            //intake.setIntakeState(false);
            //intake.update();

            shooter.closeBlocker(); // do not shoot until velocity is reached
            shooter.setVelocity("Low");

            delayTimer[3] = timer.milliseconds(); // Set Door Delay Timer for shooting
            delayTimer[4] = timer.milliseconds(); // Set Sorter Delay Timer for Shooting
        }
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)

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

        // Manage number of balls in the sorter and determine when to rotate ball to shooter position
        manageSorter();

        //Intake Control
        //manageIntake();

        // Set RGB Indicator Color by ball count
        sorter.setIndicatorLightColor();

        // Shoot Artifact without holding the thread if triggered
        if (shootArtifactAtHighSpeed || shootArtifactAtLowSpeed || shootArtifactAtCustomSpeed){
            Log.d("Shooter: ", "Shoot Artifact Called");
            shootArtifact();
        }

        //Autonomous Code:
        switch(pathState){
            case 0:
                shooter.setVelocity("High");
                follower.followPath(farShotPoint.get(), true);
                pathState = 1;
                pathChainTimer = timer.milliseconds();
                shotParametersComputed = false;
                break;

            case 1:
                if(!follower.isBusy() || (timer.milliseconds() - pathChainTimer) >= 3000) {
                    if (!shootArtifactAtHighSpeed) {
                        this.farShot();
                    }
                    if (!isShooting && !shootArtifactAtHighSpeed) {
                        pathState = 2;
                    }
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    this.toggleIntake();
                    follower.followPath(firstCollectionChain_0.get(), true);
                    pathState = 3;
                    pathChainTimer = timer.milliseconds();
                }
                break;

            case 3:
                if(!follower.isBusy() || (timer.milliseconds() - pathChainTimer) >= 3000){
                    follower.followPath(firstCollectionChain_1.get(), 1, true);
                    pathState = 4;
                    pathChainTimer = timer.milliseconds();
                    delayTimer[5] = timer.milliseconds();
                }
                break;

            case 4:
                if ((!follower.isBusy() && timerExpired(5,1000)) || (timer.milliseconds() - pathChainTimer) >= 3000) {
                    shooter.setVelocity("High");
                    follower.followPath(farShotPoint.get(), true);
                    pathState = 5;
                    pathChainTimer = timer.milliseconds();
                    shotParametersComputed = false;
                }
                break;
            case 5:
                if(!follower.isBusy() || ((timer.milliseconds() - pathChainTimer) >= 1200 && (timer.milliseconds() - pathChainTimer) <= 3000)){
                    intake.slapArtifact();
                }
                if(!follower.isBusy() || (timer.milliseconds() - pathChainTimer) >= 3000){
                    if (!shootArtifactAtHighSpeed){
                        this.farShot();
                    }
                    if (!isShooting && !shootArtifactAtHighSpeed){
                        intake.unslapArtifact();
                        pathState = 10;
                    }
                }
                break;

//            case 6:
//                if(!follower.isBusy()) {
//                    shooter.setVelocity("Low");
//                    toggleIntake();
//                    follower.followPath(secondCollectionChain_0.get(), true);
//                    pathState = 7;
//                }
//                break;
//
//            case 7:
//                if(!follower.isBusy()) {
//                    follower.followPath(secondCollectionChain_1.get(), 1, true);
////                    Log.d("StateReached: ", "State 7");
//                    pathState = 8;
//                    delayTimer[5] = timer.milliseconds();
//                }
//                break;
//
//            case 8:
//                if(!follower.isBusy()) {
//                    shooter.setVelocity("Low");
//                    follower.followPath(closeShotPoint.get(), true);
//                    pathState = 9;
//                    shotParametersComputed = false;
//                }
//                break;
//
//            case 9:
//                if(!follower.isBusy()) {
//                    if (!shootArtifactAtCustomSpeed) {
//                        this.customShot();
//                    }
//                    if (!isShooting && !shootArtifactAtCustomSpeed) {
//                        pathState = 10;
//                    }
//                }
//                break;

            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(endingChain.get(), 1.0, true);
//                    Log.d("StateReached: ", "State 7");
                    pathState = -1;
                    delayTimer[5] = timer.milliseconds();
                }
                break;


        }

        stack = sorter.getArtifactStack();
//        telemetry.addData("IsAutomatedDriveMode: ", "" + automatedDrive);
//        telemetry.addData("SorterFull: ", sorter.isFull());

        telemetry.addData("ShooterFrontVel t/s: ", shooter.getShooterFrontVel());
        telemetry.addData("ShooterBackVel t/s: ", shooter.getShooterBackVel());

//        telemetry.addData("SorterPos: ", sorter.getPosition());

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addData("Current State", pathState);
//        telemetry.addData("ShootArtifactAtLowSpeed", shootArtifactAtLowSpeed);
//        telemetry.addData("DelayTimer[3]", delayTimer[3]);
//        telemetry.addData("DelayTimer[3] duration", timer.milliseconds() - delayTimer[3]);

        telemetry.addLine("--------CUSTOM SHOT PARAMS-----------");
        telemetry.addData("Custom Shooter Vel t/s", customParameters[0]);
        telemetry.addData("Custom Shooting Heading in Degrees", customParameters[1]);


    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}