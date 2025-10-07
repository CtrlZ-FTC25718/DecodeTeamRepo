package pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MappedActuators;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "TestAuto_PP")
public class TestAuto_PP extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean parallelExecuteCaseProtect;

    //Task Class:
    private MappedActuators taskActuators;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    //Robot Poses:
    private static final Pose startPose = new Pose(0, 0 ,0);
    private static final Pose frontPose = new Pose(24 ,0, Math.toRadians(-45));
    private static final Pose backPose = new Pose(-24, 0, 0);
    private static final Pose rightPose = new Pose(0, -24, 0);
    private static final Pose leftPose = new Pose(0, 24 ,0);

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain toFront, toBack, toRight, toLeft, toStart;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        // Build Paths:
        toFront = follower.pathBuilder().addPath(new BezierLine(startPose, frontPose)).setLinearHeadingInterpolation(0, Math.toRadians(-45)).build();
        toBack = follower.pathBuilder().addPath(new BezierLine(frontPose, backPose)).setLinearHeadingInterpolation(Math.toRadians(-45), 0).build();
        toRight = follower.pathBuilder().addPath(new BezierLine(backPose, rightPose)).setConstantHeadingInterpolation(0).build();
        toLeft = follower.pathBuilder().addPath(new BezierLine(rightPose, leftPose)).setConstantHeadingInterpolation(0).build();
        toStart = follower.pathBuilder().addPath(new BezierLine(leftPose, startPose)).setConstantHeadingInterpolation(0).build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()){
                    follower.followPath(toFront, true); //Hold-end needed for blocking non-parallel functionality
                    setPathState(1);
                }
                break;


            case 1:
                if(!follower.isBusy()) {

                    //Blocking Function Sequential Example:
//                    taskActuators.taskServoOne.setPosition(1);
//                    Thread.sleep(1000);
//                    taskActuators.taskServoOne.setPosition(0);
//                    Thread.sleep(1000);

                    follower.followPath(toBack);
                    setPathState(2);
                }
                break;
            case 2:

                //Non-Blocking Function Parallel Ex:
                if(pathTimer.getElapsedTimeSeconds() > .25 && !parallelExecuteCaseProtect){
//                    taskActuators.taskServoTwo.setPosition(1);
                    parallelExecuteCaseProtect = true;
                }
                if(!follower.isBusy()) {

                    follower.followPath(toRight);
                    setPathState(3);

                    //Restore Case Protect Variable for Later Re-use:
                    parallelExecuteCaseProtect = false;
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    //Non-Blocking Function Sequential Ex:
//                    taskActuators.taskServoTwo.setPosition(0);
                    follower.followPath(toLeft);
                    setPathState(4);
//                    taskActuators.taskServoOne.setPosition(1);
                }
                break;
            case 4:

                //Normally Blocking Function Parallel Ex:

                if(pathTimer.getElapsedTimeSeconds() > 1 && !parallelExecuteCaseProtect){
//                    taskActuators.taskServoOne.setPosition(0);
                    parallelExecuteCaseProtect = true;
                }
                //If parallel task HAS NOT elapsed but the movement is complete:
                else if(!follower.isBusy() && !parallelExecuteCaseProtect){
                    while(pathTimer.getElapsedTimeSeconds() < 1){/* Do nothing */} // Block Thread Execution until parallel task is ready to be completed
//                    taskActuators.taskServoOne.setPosition(0);
                    parallelExecuteCaseProtect = true;
                }

                //If parallel task HAS elapsed AND the movement is complete:
                if(!follower.isBusy() && parallelExecuteCaseProtect /*Replace w/ completion check*/) {
                    follower.followPath(toStart, true);
                    setPathState(0);

                    //Restore Case Protect Variable for Later Re-use:
                    parallelExecuteCaseProtect = false;
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();

        taskActuators = new MappedActuators(hardwareMap);

        parallelExecuteCaseProtect = false;
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

