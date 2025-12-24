package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.lights.RGBIndicator;
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
    public Pose startingPose  = new Pose(48,9, Math.toRadians(90));; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
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
    private boolean shootArtifactAtHighSpeed, shootArtifactAtLowSpeed, shootArtifactAtCustomSpeed;

    double[] targetPos = new double [4]; // Holds nRedTargetX, RedTargetY, BlueTargetX, BlueTargetY

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(72, 24))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(65), 2))
                .build();

        intake = new Intake(hardwareMap);
        sorter = new Sorter(hardwareMap);
        shooter = new Shooter(hardwareMap);

        timer = new ElapsedTime();
        sorter.shift(0);
        sorter.door("Close");
        sorter.update();

        sorter.initializeIndicatorColor();


        shootArtifactAtHighSpeed = false;
        shootArtifactAtLowSpeed = false;
        shootArtifactAtCustomSpeed = false;
        shotCount = 0;

        // Set Target positions
        targetPos[0] = 140; // Red Target X
        targetPos[1] = 140; // Red Target Y
        targetPos[2] = 0; // Blue Target X
        targetPos[3] = 140; // Blue Target Y

        // Use shotTimer to delay for various reasons\
        // Index 0 used for delaying intake reversal when sorter is full
        // Index 1 used for delaying sorter rotation for intake
        // Index 2 used for delaying ball detection
        // Index 3 used for delaying for door opening to shoot artifact
        // Index 4 used for delaying sorter rotation for shooting
        delayTimer = new double[5]; // create 4 delayTimers that can used for various (non-blocking) delays; sets to 0.0s


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
        if (timerExpired(3,2000)){
            //Log.d("Shooter1", "Sorter Door Timer Expired");

            if(!sorter.isEmpty()){
                Log.d("Shooter2", "Sorter Not empty, waiting for timer 4 to expire");
                if  (timerExpired(4, 750)) {
                    Log.d("Shooter3", "Sorter Timer Expired");

                    if (sorter.hasDoorOpened()) {
                        //Log.d("Shooter4", "Sorter Door Is Open");

                        // Long shot
                        if (shootArtifactAtHighSpeed) {

                            //Log.d("Shooter5", "Long shot active");
                            if (shooter.isAtHighVel()) {
                                //shooter.velocityHold(0.1);
                                Log.d("Shooter5p1", "Shooter reached high Vel");

                                if (!sorter.isEmpty() && shotCount > 1) { //First artifact is already in the shooter when the door opened
                                    sorter.shift(1);
                                    sorter.update();
                                    //Log.d("ShooterShift", "Sorter Has Shifted");
                                }
                                shooter.openBlocker();

                                stack = sorter.getArtifactStack();
                                Log.d("ShootingStackBefore", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);
                                sorter.registerShot();
                                stack = sorter.getArtifactStack();
                                Log.d("ShootingStackAfter", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);

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
                            double [] customVelocities = shooter.computeProjectileMotion(follower.getPose().getX(), follower.getPose().getY(),140, 140);
                            if (shooter.isAtCustomVel(customVelocities[0], customVelocities[0])) {
                                //shooter.velocityHold(0.1);
                                Log.d("Shooter5p1", "Shooter reached custom vel");

                                if (!sorter.isEmpty() && shotCount > 1) { //First artifact is already in the shooter when the door opened
                                    sorter.shift(1);
                                    sorter.update();
                                    //Log.d("ShooterShift", "Sorter Has Shifted");
                                }
                                shooter.openBlocker();

                                stack = sorter.getArtifactStack();
                                Log.d("ShootingStackBefore", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);
                                sorter.registerShot();
                                stack = sorter.getArtifactStack();
                                Log.d("ShootingStackAfter", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);

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
                if (timerExpired(4, 2000)){
                    // Sorter is empty; But will execute until shootArtifactAtHighSpeed is false
                    // Execute end actions after shooting artifacts
                    //Log.d("Shooter14", "Sorter Empty End Shooting and Reset Timers");
                    sorter.door("Close");
                    //sorter.shift(-3);
                    sorter.reset();
                    sorter.update();

                    shooter.setVelocity("Idle",0,0); // Ignore customVel for idle
                    //shooter.closeBlocker();

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

    public void manageIntake(){
        //Set Intake Direction Based Upon Space:
        if(sorter.isFull()){
            if (timerExpired(0,5000)) {
                intake.setPower(0); //Set intake to reject
                //shooter.setPower(1, 1);
                intake.update();
            }
        }
        else{
            intake.setPower(1);
            intake.update();
            delayTimer[0] = 0; // Reset intake reverse timer
        }

    }

    public void manageSorter(){
        if(sorter.isFull() && delayTimer[0] == 0){
            delayTimer[0] = timer.milliseconds(); // Start a new timer to stop the intake and reject artifacts
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
                }
                if(timerExpired(1, 100)){
                    sorter.shift(2); //If ball has been taken in & sorter not full, shift to shooting pos [0]
                    sorter.update();
                    delayTimer[1] = 0;
                    delayTimer[2] = timer.milliseconds();
                    //Log.d("Shifting2", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);
                }
            }
        }

    }
    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        shooter.setVelocity("Idle", 0, 0);// Ignore customVels
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
        telemetry.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp

            double rsy = gamepad1.right_stick_y;
            double rsx = gamepad1.right_stick_x;
            double lsx = gamepad1.left_stick_x;

            //This is how it looks with slowMode on
            // Scale normal driving as a quadratic X^2
            if (!slowMode) {

                if (rsy > 0){ rsy = -Math.pow(rsy,2);}
                else {rsy = Math.pow(rsy,2);}

                if (rsx > 0){ rsx = -Math.pow(rsx,2);}
                else {rsx = Math.pow(rsx,2);}

                if (lsx > 0){ lsx = -Math.pow(lsx,2);}
                else {lsx = Math.pow(lsx,2);}

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

        // Set RGB Indicator Color by ball count
        sorter.setIndicatorLightColor();

        // Start or Stop intake based upon Sorter State
        manageIntake();

        // Manage number of balls in the sorter and determine when to rotate ball to shooter position
        manageSorter();

        // Shoot Artifact without holding the thread if triggered
        if (shootArtifactAtHighSpeed || shootArtifactAtLowSpeed || shootArtifactAtCustomSpeed){
            shootArtifact();
        }

        // Handle/Respond to button clicks
        // Go to Long Shot artifact shooting position
        if (gamepad1.a || gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get(),true);
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

        if(gamepad1.yWasPressed() || gamepad1.yWasReleased()){
            //Shoot Artifacts (everything that is in the stack)
            if(!sorter.isEmpty()){
                shootArtifactAtHighSpeed = true;
                shooter.velocityHold("High", .1, 0, 0); // initial spinup; ignore custVel inputs for High vel
                shotCount = 0;
                sorter.door("Open");
                sorter.update();
                sorter.wiggleUp();
                intake.setPower(0);
                intake.update();
                shooter.closeBlocker(); // do not shoot until velocity is reached
                shooter.setVelocity("High",0,0); // Ignore custVel for High vel

                delayTimer[3] = timer.milliseconds(); // Set Door Delay Timer for shooting
                delayTimer[4] = timer.milliseconds(); // Set Sorter Delay Timer for Shooting
            }
        }

        if(gamepad1.xWasPressed() || gamepad1.xWasReleased()){
            //Shoot Artifacts (everything that is in the stack)
            if(!sorter.isEmpty()){
                shootArtifactAtLowSpeed = true;
                shooter.velocityHold("Low", .1, 0, 0); // Initial Spinup; Ignore custVel for Low
                shotCount = 0;
                sorter.door("Open");
                sorter.update();
                sorter.wiggleUp();
                intake.setPower(0);
                intake.update();

                shooter.closeBlocker(); // do not shoot until velocity is reached
                shooter.setVelocity("Low", 0,0); // ignore custVel for Low

                delayTimer[3] = timer.milliseconds(); // Set Door Delay Timer for shooting
                delayTimer[4] = timer.milliseconds(); // Set Sorter Delay Timer for Shooting
            }
        }

        if(gamepad1.bWasPressed()){
            intake.setPower(1);
            intake.update();
        }

        if(gamepad2.dpadLeftWasPressed()){
            sorter.setPosition(sorter.getPosition()+0.0025);
        }
        if(gamepad2.dpadRightWasPressed()){
            sorter.setPosition(sorter.getPosition()-0.0025);
        }

        if(gamepad2.dpadUpWasPressed()){
            sorter.setPosition(sorter.getPosition()+0.01);
        }
        if(gamepad2.dpadDownWasPressed()){
            sorter.setPosition(sorter.getPosition()-0.01);
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

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}