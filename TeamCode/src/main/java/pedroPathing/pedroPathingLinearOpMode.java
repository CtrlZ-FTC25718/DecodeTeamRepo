package pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;

import java.util.function.Supplier;

@Autonomous
public class pedroPathingLinearOpMode extends LinearOpMode {

    private LinearFollower follower;
    private Timer opmodeTimer;
    private Servo taskServo;

    private int taskOneLoopCount;

    public boolean eqWT(double val1, double val2, double e){
        return Math.abs(val1 - val2) <= e;
    }

    @Override
    public void runOpMode(){

        //Initialize Pedro Pathing
        follower = new LinearFollower(hardwareMap);
        taskServo = hardwareMap.get(Servo.class, "s2");
        taskOneLoopCount = 0;

        //Define Paths
        PathChain firstMove = follower.pathBuilder()
                                      .addPath(new BezierLine(new Point(0, 0), new Point(24, 0)))
                                      .setConstantHeadingInterpolation(0)
                                      .build();

        PathChain secondMove = follower.pathBuilder()
                                       .addPath(new BezierLine(new Point(24, 0), new Point(24, -24)))
                                       .setConstantHeadingInterpolation(0)
                                       .build();

        PathChain thirdMove = follower.pathBuilder()
                                      .addPath(new BezierCurve(new Point(24, -24), new Point(0, -24), new Point(0, 0)))
                                      .setConstantHeadingInterpolation(0)
                                      .build();

        //Define Tasks
        Runnable taskOneLooping = () -> {
            if(eqWT(taskServo.getPosition(), 0, .05)){
                taskServo.setPosition(1);
            }
            if(eqWT(taskServo.getPosition(), 1, .05)){
                taskServo.setPosition(0);
            }
        };

        Supplier<Boolean> taskOneExit = () -> {return taskOneLoopCount < 1000;};

        //Wait for the start signal
        waitForStart();

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower.setStartingPose(new Pose(0, 0, 0));

        if(opModeIsActive()){

           for(int i = 0; i < 3; i++){
                //Execute first path
                follower.followPath(firstMove);

                //Execute second path
                follower.followPath(secondMove);

                //Execute third path
                follower.setParallelTask(taskOneLooping, true);
                follower.followPath(thirdMove);
            }

        }
    }
}
