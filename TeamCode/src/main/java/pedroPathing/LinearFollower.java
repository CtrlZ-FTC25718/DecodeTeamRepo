package pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Supplier;

public class LinearFollower extends Follower {

    private Timer pathTimer;
    private Runnable currentNonLoopingParallelTask;
    private Runnable currentLoopingParallelTask;
    private Supplier<Boolean> holdExitFunction;

    public LinearFollower(HardwareMap hardwareMap){
        super(hardwareMap, pedroPathing.constants.FConstants.class, pedroPathing.constants.LConstants.class);
        pathTimer = new Timer();
        currentNonLoopingParallelTask = () -> {}; // Default empty lambda function
        currentLoopingParallelTask = () -> {}; // Default empty lambda function
        holdExitFunction = () -> {return true;}; // Default lambda function
    }

    //Use of a nonLooping task is only in the holdState variation of the followPath function
    public void setParallelTask(Runnable parallelTask, boolean isLooping){
        if(isLooping){
            currentLoopingParallelTask = parallelTask;
        }
        else{
            currentNonLoopingParallelTask = parallelTask;
        }
    }

    public void setHoldExitFunction(Supplier<Boolean> exitFunction){
        holdExitFunction = exitFunction;
    }

    @Override
    public void followPath(PathChain path){
        pathTimer.resetTimer();
        super.followPath(path);
        while(super.isBusy()){
            super.update();
            currentLoopingParallelTask.run();
        }
    }

    //Note: Never use the followPath(PathChain path, boolean holdState) method of the superclass()
    //Second Note: The Runnable argument is the parallel looping task for the movement; for the hold functionality, first call the setter function separately
    public void followPath(PathChain path, boolean holdState, Runnable movementParallelTask){
        pathTimer.resetTimer();
        super.followPath(path);
        while(super.isBusy()){
            super.update();
            movementParallelTask.run();
        }
        if(holdState){
            holdPoint(super.getPose());
        }
    }

    @Override
    public void followPath(Path path){
        pathTimer.resetTimer();
        super.followPath(path);
        while(super.isBusy()){
            super.update();
            currentLoopingParallelTask.run();
        }
    }

    @Override
    public void holdPoint(Pose target){
        pathTimer.resetTimer();
        currentNonLoopingParallelTask.run();
        PathChain returnPath;
        while(this.holdExitFunction.get()){
            returnPath = super.pathBuilder()
                              .addPath(new BezierLine(new Point(super.getPose()), new Point(target)))
                              .setLinearHeadingInterpolation(super.getPose().getHeading(), target.getHeading())
                              .build();
            super.followPath(returnPath);
            currentLoopingParallelTask.run();
        }
    }


}
