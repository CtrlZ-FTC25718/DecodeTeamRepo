package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Log;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Sorter;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "ShooterPIDTuning")
public class ShooterPIDTuning extends OpMode {

    private TelemetryManager telemetryM;
    private Sorter sorter;
    private Intake intake;
    private Shooter shooter;
    private DcMotorEx front;
    private DcMotorEx back;

    private double highVelocity = 1800;
    private double lowVelocity = 0;

    private double targetVelocity = lowVelocity;

    private double currFrontTargetVelocity = lowVelocity;
    private double currBackTargetVelocity = lowVelocity;

    private double frontF = 16.4;//15.106
    private double frontP = 60;//46.601, then 300
    private double frontI = 0;
    private double frontD = 0;
    private double backF = 20.6;
    private double backP = 50;
    private double backI = 0;
    private double backD = 0;

    private PIDFCoefficients frontPIDF;
    private PIDFCoefficients backPIDF;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        sorter = new Sorter(hardwareMap);
        shooter = new Shooter(hardwareMap);

        front = shooter.getShooterFrontMotor();
        back = shooter.getShooterBackMotor();

        frontPIDF = new PIDFCoefficients(frontP,frontI,frontD,frontF);
        backPIDF = new PIDFCoefficients(backP,backI,backD,backF);

        front.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, frontPIDF);
        back.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, backPIDF);
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
           }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        telemetry.update();

        if (gamepad1.yWasPressed()){
            if (currFrontTargetVelocity == highVelocity){
                currFrontTargetVelocity = lowVelocity;
            }
            else {
                currFrontTargetVelocity = highVelocity;
            }
        }
        if (gamepad2.yWasPressed()){
            if (currBackTargetVelocity == highVelocity){
                currBackTargetVelocity = lowVelocity;
            }
            else {
                currBackTargetVelocity = highVelocity;
            }
        }

        // Front

        if(gamepad1.bWasPressed() || gamepad2.bWasPressed()){
            stepIndex = (stepIndex+1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            frontP -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()){
            frontP += stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()){
            frontF += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()){
            frontF -= stepSizes[stepIndex];
        }
        if (gamepad1.left_trigger>0){
            frontI -= stepSizes[stepIndex];
        }
        if (gamepad1.right_trigger>0){
            frontI += stepSizes[stepIndex];
        }
        if (gamepad1.leftBumperWasPressed()){
            frontD -= stepSizes[stepIndex];
        }
        if (gamepad1.rightBumperWasPressed()){
            frontD += stepSizes[stepIndex];
        }



        // Back

        if (gamepad2.dpadLeftWasPressed()){
            backP -= stepSizes[stepIndex];
        }
        if (gamepad2.dpadRightWasPressed()){
            backP += stepSizes[stepIndex];
        }
        if (gamepad2.dpadUpWasPressed()){
            backF += stepSizes[stepIndex];
        }
        if (gamepad2.dpadDownWasPressed()){
            backF -= stepSizes[stepIndex];
        }

        if (gamepad2.left_trigger>0){
            backI -= stepSizes[stepIndex];
        }
        if (gamepad2.right_trigger>0){
            backI += stepSizes[stepIndex];
        }
        if (gamepad2.leftBumperWasPressed()){
            backD -= stepSizes[stepIndex];
        }
        if (gamepad2.rightBumperWasPressed()){
            backD += stepSizes[stepIndex];
        }

        frontPIDF.p = frontP;
        frontPIDF.f = frontF;
        frontPIDF.i = frontI;
        frontPIDF.d = frontD;

        backPIDF.p = backP;
        backPIDF.f = backF;
        backPIDF.i = backI;
        backPIDF.d = backD;


        front.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,frontPIDF);
        back.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,backPIDF);

        front.setVelocity(currFrontTargetVelocity);
        back.setVelocity(currBackTargetVelocity);

        double frontCurrVelocity = front.getVelocity();
        double backCurrVelocity = back.getVelocity();

        double frontError = currFrontTargetVelocity - frontCurrVelocity;
        double backError = currBackTargetVelocity - backCurrVelocity;

        telemetry.addData("StepSize : ", stepSizes[stepIndex]);
        telemetry.addData("Target Velocity : ", targetVelocity);
        telemetry.addLine("***************************************************");
        telemetry.addData("Current Front Velocity : ", frontCurrVelocity);
        telemetry.addData("Current Front Error : ", frontError);
        telemetry.addLine("-------------------------------------------");
        telemetry.addData("Current Back Velocity : ", backCurrVelocity);
        telemetry.addData("Current Back Error : ", backError);
        telemetry.addLine("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
        telemetry.addData("Tuning Front P : ", "%.4f", frontP);
        telemetry.addData("Tuning Front I : ", "%.4f", frontI);
        telemetry.addData("Tuning Front D : ", "%.4f", frontD);
        telemetry.addData("Tuning Front F : ", "%.4f", frontF);
        telemetry.addData("Tuning Back P : ", "%.4f", backP);
        telemetry.addData("Tuning Back I : ", "%.4f", backI);
        telemetry.addData("Tuning Back D : ", "%.4f", backD);
        telemetry.addData("Tuning Back F : ", "%.4f", backF);
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}