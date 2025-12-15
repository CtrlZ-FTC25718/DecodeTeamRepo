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
    private double[] delayTimer;
    private String[] stack;
    private boolean shootArtifact;

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

        shootArtifact = false;
        // Use shotTimer to delay for various reasons
        // Index 0, 1, 2 used for shot delay management
        // Index 3 used for delaying intake reversal when sorter is full
        // Index 4 used for delaying sorter shift once a ball has been detected
        delayTimer = new double[5]; // create 5 delayTimers that can used for various (non-blocking) delays; sets to 0.0s


    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    // General timerExpiration Check function
    private boolean timerExpired(int timerIndex , double waitTime){
        if (timer.milliseconds() - delayTimer[timerIndex] < waitTime){
            return true;
        }
        else {
            return false;
        }
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
        if(sorter.isFull() && timerExpired(3,750)){
            intake.setPower(-1); //Set intake to reject
            shooter.setPower(1, 1);
            intake.update();
        }

        else{
            intake.setPower(1);
            intake.update();
        }

        sorter.detect(); //Detect potential artifact
        stack = sorter.getArtifactStack();

        if(sorter.isFull() && delayTimer[3] == 0){
            delayTimer[3] = timer.milliseconds(); // Start a new timer to stop the intake and reject artifacts
        }

        if(!stack[2].equals("")){
            if(!sorter.isFull()){
                sorter.shift(2); //If ball intaked & not full, shift to shooting pos [0]

                if(delayTimer[4] == 0){
                    delayTimer[4] = timer.milliseconds(); // Start a new timer to wait to rotate the sorter
                }
                if(timerExpired(4, 500)){
                    sorter.update();
                }
            }
        }


        // Shoot Artifact without holding the thread
        if (shootArtifact){
            stack = sorter.getArtifactStack();
            if(!sorter.isEmpty()){
                if (sorter.hasDoorOpened() && timerExpired(0,1000) ) {
                    sorter.registerShot();
                    sorter.shift(1);
                    sorter.update();
                    // re-set delayTimer[0]
                    delayTimer[0] = timer.milliseconds(); // Reset for waiting between shots
                    delayTimer[1] = timer.milliseconds(); // Reset timer for after sorter is empty and end of shooting actions
                }
            }
            else {
                // Sorter is empty; But will execute until shootArtifact is false
                // Execute end actions after shooting artifacts
                shooter.setPower(0, 0);
                sorter.door("Close");
                sorter.update();

                // Wait for door to close
                if (timerExpired (1,750)) {
                    sorter.shift(-3);
                    sorter.update();
                    delayTimer[2] = timer.milliseconds(); // Reset timer for final delay in end of shooting actions
                }
                if (timerExpired(2, 1000)){
                    // All done with shooting artifacts
                    shootArtifact = false;

                    // Reset all timers used for shooting artifacts
                    for (int i = 0; i<3; i++){
                        delayTimer[i] = 0.0;
                    }

                    // Reset Intake Timer delayTimer[3]
                    delayTimer[3] = 0.0;
                }
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


        if(gamepad1.xWasPressed() || gamepad1.xWasReleased()){
            //Shoot Artifacts (everyting that is in the stack)
            if(!sorter.isEmpty()){
                shootArtifact = true;
                shooter.setPower(1, 1);
                intake.setPower(0);
                intake.update();
                //this.delay(3000);
                sorter.door("Open");
                shooter.openBlocker();
                sorter.update();
                // initialize delayTimer[0]
                delayTimer[0] = timer.milliseconds();

            }
        }

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