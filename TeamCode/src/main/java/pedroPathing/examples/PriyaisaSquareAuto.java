package pedroPathing.examples;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Autonomous(name = "PriyaisaSquareAuto", group = "Examples")
public class PriyaisaSquareAuto extends OpMode {
    private Follower follower;

    private Timer pathTimer;
    private Timer opmodeTimer;

    private int pathState;

    Pose startPose = new Pose(0, 0, 0);
    Pose firstCorner = new Pose(24, -12, -Math.toRadians(45));
    Pose secondCorner = new Pose(48, 0, Math.toRadians(45));
    Pose thirdCorner = new Pose(24, 24, Math.toRadians(135));

    private Path firstSide, secondSide, thirdSide, fourthSide;
    private PathChain firstChain, secondChain, thirdChain, fourthChain;

    public void buildPaths(){

        /*firstSide = new Path(new BezierLine(new Point(0, 0), new Point(24, 0)));
        firstSide.setLinearHeadingInterpolation(0, Math.toRadians(45));*/

        // firstSide = new Path(new BezierLine(new Point(startPose), new Point(firstCorner)));
        firstSide = new Path(new BezierCurve(new Point(startPose), new Point(firstCorner)));
        firstSide.setLinearHeadingInterpolation(startPose.getHeading(), firstCorner.getHeading());

        secondSide = new Path(new BezierCurve(new Point(firstCorner), new Point(secondCorner)));
        secondSide.setLinearHeadingInterpolation(firstCorner.getHeading(), secondCorner.getHeading());

        thirdSide = new Path(new BezierCurve(new Point(secondCorner), new Point(thirdCorner)));
        thirdSide.setLinearHeadingInterpolation(secondCorner.getHeading(), thirdCorner.getHeading());

        fourthSide = new Path(new BezierLine(new Point(thirdCorner), new Point(startPose)));
        fourthSide.setLinearHeadingInterpolation(thirdCorner.getHeading(), startPose.getHeading());


        firstChain = follower.pathBuilder().addPath(firstSide).build();
        secondChain = follower.pathBuilder().addPath(secondSide).build();
        thirdChain = follower.pathBuilder().addPath(thirdSide).build();
        fourthChain = follower.pathBuilder().addPath(fourthSide).build();

    }

    public boolean eqWT(double val1, double val2, double e){
        return Math.abs(val1 - val2) <= e;
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                if(!follower.isBusy()){
                    follower.followPath(firstChain);
                    setPathState(1);
                }

                break;

            case 1:

                if(!follower.isBusy()){
                    follower.followPath(secondChain);
                    setPathState(2);
                }

                break;

            case 2:

                if(!follower.isBusy()){
                    follower.followPath(thirdChain);
                    setPathState(3);
                }

                break;

            case 3:

                if(!follower.isBusy()){
                    follower.followPath(fourthChain, true);
                    setPathState(4);
                }

                break;

            case 4:

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

        follower.update();
        autonomousPathUpdate();
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
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

















































//package org.firstinspires.ftc.teamcode.pedroPathing.examples;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.follower.FollowerConstants;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.Path;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//
///**
// * This is an example auto that showcases movement and control of two servos autonomously.
// * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
// * There are examples of different ways to build paths.
// * A path progression method has been created and can advance based on time, position, or other factors.
// *
// * @author Baron Henderson - 20077 The Indubitables
// * @version 2.0, 11/28/2024
// */
//
//@Autonomous
//public class PriyaisaSquareAuto extends OpMode {
//
////    private Servo taskServo;
//
//    private Follower follower;
//
//    private Timer pathTimer;
//    private Timer opmodeTimer;
//
//    private int pathState;
//
//    //Key Robot Positions:
//    Pose startPose = new Pose(0, 0, 0);
//    Pose firstCorner = new Pose(24, 0, Math.toRadians(0));
//    Pose secondCorner = new Pose(24, 24, -Math.toRadians(0));
//    Pose thirdCorner = new Pose(0, 24, Math.toRadians(0));
//    Pose finalTurn = new Pose(0, 24 , Math.toRadians(90));
//
//    private Path firstSide, secondSide, thirdSide, fourthSide, turn, fifthSide, sixthSide, megaSide, reverseFirstSide, strafeSide, seventhSide, reverseSeventhSide, curve;
//    private PathChain firstChain, secondChain, thirdChain, fourthChain, turnChain, betterFirstChain, betterSecondChain,aChain,bChain, megaChain, fat1Chain, fat2Chain,  fat3Chain, curveChain;
//
//
//
//    public void buildPaths(){
//
//        /*firstSide = new Path(new BezierLine(new Point(0, 0), new Point(24, 0)));
//        firstSide.setLinearHeadingInterpolation(0, Math.toRadians(45));*/
//
//       // firstSide = new Path(new BezierLine(new Point(startPose), new Point(firstCorner)));
//        firstSide = new Path(new BezierCurve(new Point(startPose), new Point(firstCorner)));
//        firstSide.setLinearHeadingInterpolation(startPose.getHeading(), firstCorner.getHeading());
//
//        secondSide = new Path(new BezierCurve(new Point(firstCorner), new Point(secondCorner)));
//        secondSide.setLinearHeadingInterpolation(Math.toRadians(45), -Math.toRadians(45));
//
//        thirdSide = new Path(new BezierCurve(new Point(secondCorner), new Point(thirdCorner)));
//        thirdSide.setLinearHeadingInterpolation(-Math.toRadians(45), Math.toRadians(45));
//
//        fourthSide = new Path(new BezierLine(new Point(thirdCorner), new Point(startPose)));
//        fourthSide.setLinearHeadingInterpolation(Math.toRadians(45), 0);
//
//
//        fifthSide = new Path(new BezierCurve(new Point(startPose),new Point(firstCorner), new Point(secondCorner)));
//    //    fifthSide.setLinearHeadingInterpolation(startPose.getHeading(),secondCorner.getHeading());
//        fifthSide.setTangentHeadingInterpolation();
//
//        sixthSide = new Path(new BezierCurve(new Point(secondCorner), new Point(thirdCorner), new Point(startPose)));
//       // sixthSide.setLinearHeadingInterpolation(secondCorner.getHeading(),startPose.getHeading());
//        sixthSide.setTangentHeadingInterpolation();
//
//        megaSide = new Path(new BezierCurve(new Point(startPose),new Point(firstCorner), new Point(secondCorner), new Point(thirdCorner), new Point(startPose)));
//        megaSide.setTangentHeadingInterpolation();
//
//        reverseFirstSide = new Path(new BezierLine(new Point(firstCorner), new Point(startPose)));
//        reverseFirstSide.setLinearHeadingInterpolation(0,0);
//
//        strafeSide = new Path(new BezierLine(new Point(startPose), new Point(thirdCorner)));
//        strafeSide.setLinearHeadingInterpolation(0,0);
//
//        seventhSide = new Path(new BezierLine(new Point(thirdCorner), new Point(secondCorner)));
//        seventhSide.setLinearHeadingInterpolation(0,0);
//
//        reverseSeventhSide = new Path(new BezierLine(new Point(secondCorner), new Point(thirdCorner)));
//        reverseSeventhSide.setLinearHeadingInterpolation(0,0);
//
//        curve = new Path(new BezierCurve(new Point(firstCorner), new Point(startPose), new Point(thirdCorner)));
//        curve.setLinearHeadingInterpolation(0,0);
//
//        turn = new Path(new BezierCurve(new Point(thirdCorner), new Point(finalTurn)));
//
//        firstChain = follower.pathBuilder().addPath(firstSide).build();
//        secondChain = follower.pathBuilder().addPath(secondSide).build();
//        thirdChain = follower.pathBuilder().addPath(thirdSide).build();
//        fourthChain = follower.pathBuilder().addPath(fourthSide).build();
//        turnChain = follower.pathBuilder().addPath(turn).build();
//
//
//        betterFirstChain = follower.pathBuilder().addPath(firstSide).addPath(secondSide).build();
//        betterSecondChain = follower.pathBuilder().addPath(thirdSide).addPath(fourthSide).build();
//        aChain = follower.pathBuilder().addPath(fifthSide).build();
//        bChain = follower.pathBuilder().addPath(sixthSide).build();
//        megaChain = follower.pathBuilder().addPath(megaSide).build();
//
//        curveChain =follower.pathBuilder().addPath(curve).build();
//
//        fat1Chain = follower.pathBuilder()
//                .addPath(firstSide)
//                .addPath(reverseFirstSide)
//                .build();
//
//        fat2Chain = follower.pathBuilder()
//                .addPath(strafeSide)
//                .build();
//
//        fat3Chain = follower.pathBuilder()
//                .addPath(seventhSide)
//                .addPath(reverseSeventhSide)
//                .build();
//    }
//
//    public boolean eqWT(double val1, double val2, double e){
//        return Math.abs(val1 - val2) <= e;
//    }
//
////    public void oscillateServo(){
////        for(int i = 0; i < 10; i++){
////            taskServo.setPosition(1);
////            while(!eqWT(taskServo.getPosition(), 1, .05)){}
////            taskServo.setPosition(0);
////            while(!eqWT(taskServo.getPosition(), 0, .05)){}
////        }
////    }
//
//    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
//     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
//     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//
//                if(!follower.isBusy()){
//                    //follower.followPath(fat1Chain, true);
//                    follower.followPath(firstSide);
////                    taskServo.setPosition(1);
//                    setPathState(1);
//                }
//
//                break;
//
//            case 1:
//
//                if(!follower.isBusy()){
//                    //follower.followPath(fat2Chain, true);
//                    follower.followPath(curveChain);
////                    taskServo.setPosition(0);
//                    setPathState(2);
//                }
//
//                break;
//
//            case 2:
//
//                if(!follower.isBusy()){
//                    follower.followPath(fat3Chain, true);
//                    follower.followPath(strafeSide);
////                    taskServo.setPosition(1);
//                    setPathState(3);
//                }
//
//                break;
//
//            case 3:
//
//                if(!follower.isBusy()){
//                    //follower.followPath(turnChain, true);
//                    //follower.followPath(fat2Chain, true);
//                    follower.followPath(seventhSide);
////                    taskServo.setPosition(0);
//                    setPathState(4);
//                }
//
//                break;
//
//            case 4:
//
//                if(!follower.isBusy()){
//                    //follower.followPath(fat3Chain, true);
//                    //oscillateServo();
//                    follower.followPath(reverseSeventhSide);
////                    taskServo.setPosition(0);
//                    setPathState(5);
//                }
//
//                break;
//
//            case 5:
//
//                if(!follower.isBusy()){
//
//                    //oscillateServo();
//                    follower.followPath(turnChain);
////                    taskServo.setPosition(1);
//                    setPathState(6);
//                }
//
//                break;
//
//            /*case 4:
//
//                if(!follower.isBusy()){
//                    follower.followPath(turnChain, true);
//                    //oscillateServo();
//                    taskServo.setPosition(0);
//                    setPathState(5);
//                }
//                break;*/
//
//            case 6:
//
//                if(!follower.isBusy()) {
//
//                    setPathState(-1);
//                }
//                break;
//        }
//    }
//
//    /** These change the states of the paths and actions
//     * It will also reset the timers of the individual switches **/
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
//    @Override
//    public void loop() {
//
//        follower.update();
//        autonomousPathUpdate();
//    }
//
//    /** This method is called once at the init of the OpMode. **/
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//
////        taskServo = hardwareMap.get(Servo.class, "s1");
//
//        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
//        //follower.setStartingPose(new Pose(0,0,0));
//        follower.setStartingPose(startPose);
//        buildPaths();
//    }
//
//    /** This method is called continuously after Init while waiting for "play". **/
//    @Override
//    public void init_loop() {}
//
//    /** This method is called once at the start of the OpMode.
//     * It runs all the setup actions, including building paths and starting the path system **/
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//    }
//
//    /** We do not use this because everything should automatically disable **/
//    @Override
//    public void stop() {
//    }
//}
