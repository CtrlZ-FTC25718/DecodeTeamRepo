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

@Autonomous(name = "MovementQuiz")
public class MovementQuiz extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private MappedActuators taskActs;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    //Robot Poses:
    Pose startPose = new Pose(0, 0, 0);
    Pose firstCorner = new Pose(48, 0, -(Math.PI-Math.atan(0.5)));
    Pose secondCorner = new Pose(0, -24, Math.toRadians(180));
    Pose thirdCorner = new Pose(48, -24, Math.toRadians(180));
    Pose finalTurn = new Pose(48, -24 , Math.toRadians(0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    Path turn;
    PathChain pC1, pC2, pC3, pC5;

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
        pC1 = follower.pathBuilder()
                      .addPath(new BezierLine(startPose, firstCorner))
                      .setLinearHeadingInterpolation(startPose.getHeading(), firstCorner.getHeading())
                      .build();

        pC2 = follower.pathBuilder()
                      .addPath(new BezierLine(firstCorner, secondCorner))
                      .setLinearHeadingInterpolation(firstCorner.getHeading(), secondCorner.getHeading())
                      .build();

        pC3 = follower.pathBuilder()
                      .addPath(new BezierLine(secondCorner, thirdCorner))
                      .setConstantHeadingInterpolation(secondCorner.getHeading())
                      .build();

        //turn = new Path(new BezierCurve(thirdCorner, finalTurn));

        pC5 = follower.pathBuilder()
                      .addPath(new BezierLine(finalTurn, startPose))
                      .setLinearHeadingInterpolation(finalTurn.getHeading(), startPose.getHeading())
                      .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()){
                    follower.followPath(pC1);
//                    taskActs.taskServoTwo.setPosition(0.5);
//                    taskActs.taskServoOne.setPosition(0.5);
                    setPathState(1);
                }
                break;

            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(pC2);
//                    taskActs.taskServoOne.setPosition(1);
                    setPathState(2);
                }
                break;
            case 2:

                if(!follower.isBusy()) {
                    follower.followPath(pC3);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    //follower.followPath(pC4);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()) {
//                    taskActs.taskServoTwo.setPosition(1);
                    follower.followPath(pC5);
                    setPathState(5);
                }
                break;

            case 5:
                if(!follower.isBusy()){
                    setPathState(0);
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
        taskActs = new MappedActuators(hardwareMap);
        //follower.setStartingPose(startPose);
        buildPaths();
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

