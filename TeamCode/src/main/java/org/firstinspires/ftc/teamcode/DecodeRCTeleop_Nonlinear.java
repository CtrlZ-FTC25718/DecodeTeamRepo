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
    public Pose startingPose  = new Pose(0,0, Math.toRadians(90));; //See ExampleAuto to understand how to use this
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
    private boolean shootArtifactAtHighSpeed;
    private boolean shootArtifactAtLowSpeed;

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

        sorter.initializeIndicatorColor();


        shootArtifactAtHighSpeed = false;
        shootArtifactAtLowSpeed = false;
        shotCount = 0;

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
                //Log.d("Shooter2", "Sorter Not empty");
                if  (timerExpired(4, 500)) {
                    //Log.d("Shooter3", "Sorter Timer Expired");
                    if (sorter.hasDoorOpened()) {
                        //Log.d("Shooter4", "Sorter Door Is Open");

                        // Long shot
                        if (shootArtifactAtHighSpeed) {

                            //Log.d("Shooter5", "Long shot active");
                            if (shooter.isAtHighVel()) {

                                shooter.openBlocker();

                                stack = sorter.getArtifactStack();
                                //Log.d("ShootingStackBefore", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);
                                sorter.registerShot();
                                stack = sorter.getArtifactStack();
                                //Log.d("ShootingStackAfter", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);

                                shotCount++;
                                if (!sorter.isEmpty()) {
                                    sorter.shift(1);
                                    sorter.update();
                                    //Log.d("ShooterShift", "Sorter Has Shifted");
                                }

                                stack = sorter.getArtifactStack();
                                //Log.d("ShootingStackAfterShift", "" + stack[0] + ", " + stack[1] + ", " + stack[2]);


                                delayTimer[4] = timer.milliseconds(); // Rest for delaying sorter next round
                                //Log.d("Shooter6", "Shot Registered at high speed: " + shotCount);
                            }
                            else {
                                //Log.d("Shooter7", "shootArtifactAtHighSpeed is false");
                            }
                        }

                        else {
                            //Log.d("Shooter11", "Velocity high/low not reached");
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

                    shooter.setVelocity("Low");
                    //shooter.closeBlocker();

                    // Done shooting set shooting states to false
                    if (shootArtifactAtHighSpeed) { shootArtifactAtHighSpeed = false;}
                    if (shootArtifactAtLowSpeed){ shootArtifactAtLowSpeed = false;}

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

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        shooter.setVelocity("Low");
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

        // Set RGB Indicator Color by ball count
        sorter.setIndicatorLightColor();

        //Set Intake Direction Based Upon Space:
        if(sorter.isFull()){
            if (timerExpired(0,3000)) {
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

        if(!stack[2].equals("") && !shootArtifactAtHighSpeed && !shootArtifactAtLowSpeed){
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

        // Shoot Artifact without holding the thread if triggered
        if (shootArtifactAtHighSpeed || shootArtifactAtLowSpeed){
            shootArtifact();
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

        if(gamepad1.yWasPressed() || gamepad1.yWasReleased()){
            //Shoot Artifacts (everything that is in the stack)
            if(!sorter.isEmpty()){
                shootArtifactAtHighSpeed = true;
                shotCount = 0;
                sorter.door("Open");
                sorter.update();
                sorter.wiggleUp();

                shooter.setVelocity("High");
                shooter.closeBlocker(); // do not shoot until velocity is reached

                intake.setPower(0);
                intake.update();

                delayTimer[3] = timer.milliseconds(); // Set Door Delay Timer for shooting
                delayTimer[4] = timer.milliseconds(); // Set Sorter Delay Timer for Shooting
            }
        }

        if(gamepad1.xWasPressed() || gamepad1.xWasReleased()){
            //Shoot Artifacts (everything that is in the stack)
            if(!sorter.isEmpty()){
                shootArtifactAtLowSpeed = true;
                shotCount = 0;
                sorter.door("Open");
                sorter.update();
                sorter.wiggleUp();

                shooter.setVelocity("Low");
                shooter.closeBlocker(); // do not shoot until velocity is reached

                intake.setPower(0);
                intake.update();

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
        telemetry.addData("Artifacts: ", '{' + stack[0] + " | " + stack[1] + " | " + stack[2] + '}');
        telemetry.addData("SorterFull: ", sorter.isFull());

        telemetry.addData("ShooterFrontVel d/s: ", shooter.getShooterFrontVel());
        telemetry.addData("ShooterBackVel d/s: ", shooter.getShooterFrontVel());

        telemetry.addData("SorterPos: ", sorter.getPosition());

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}