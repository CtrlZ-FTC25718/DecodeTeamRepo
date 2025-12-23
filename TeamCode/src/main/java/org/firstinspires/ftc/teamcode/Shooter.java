package org.firstinspires.ftc.teamcode;

//Imports
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

public class Shooter {

    //Hardware
    private DcMotorEx shooterFront;
    private DcMotorEx shooterBack;
    private Servo blocker;
    private PIDFCoefficients lowShooterFrontPIDF, highShooterFrontPIDF;
    private PIDFCoefficients lowShooterBackPIDF, highShooterBackPIDF;
    private final double[] shooterVel = {1400, 1750, 1200, 1600, 600, 800}; // In Deg/Sec: Front Low, Front High, Back Low, Back High, idle front, idle back

    //Constructor
    public Shooter(HardwareMap map) {

        shooterFront = map.get(DcMotorEx.class, "shooterFrontMotor");
        shooterBack = map.get(DcMotorEx.class, "shooterBackMotor");
        blocker = map.get(Servo.class, "shooterBlocker");

        shooterFront.setDirection(DcMotorEx.Direction.REVERSE);
        shooterFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //shooterFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterBack.setDirection(DcMotorEx.Direction.FORWARD);
        //shooterBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        lowShooterFrontPIDF = new PIDFCoefficients(400, 50, 0, 16.118);
        lowShooterBackPIDF = new PIDFCoefficients(800, 70, 70, 20);

        highShooterFrontPIDF = lowShooterFrontPIDF;
        highShooterBackPIDF = lowShooterBackPIDF;

        shooterFront.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, highShooterFrontPIDF);
        //shooterBack.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, highShooterBackPIDF);
    }

    public void setPower(double backPower, double frontPower) {
        shooterFront.setPower(frontPower);
        shooterBack.setPower(backPower);
    }

    public void velocityHold(String level, double rate){
        shooterFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //shooterBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterBack.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));

        switch (level) {
            case "High":
                shooterBack.setPower(1.0);
                while(shooterBack.getVelocity() < shooterVel[3] && shooterFront.getVelocity() < shooterVel[1] && shooterFront.getPower() != 1 && shooterBack.getPower() != 1){
                    //shooterBack.setPower(shooterBack.getPower() + rate);
                    shooterFront.setPower(shooterFront.getPower() + rate);
                }
                break;

            case "Low":
                shooterBack.setPower(0.8);
                while(shooterBack.getVelocity() < shooterVel[2] && shooterFront.getVelocity() < shooterVel[0] && shooterFront.getPower() != 1 && shooterBack.getPower() != 1){
                    //shooterBack.setPower(shooterBack.getPower() + rate);
                    shooterFront.setPower(shooterFront.getPower() + rate);
                }
                break;
        }

        shooterFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setVelocity(String level) {
        switch (level) {
            case "Low":
                shooterFront.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, lowShooterFrontPIDF);
                //shooterBack.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, lowShooterBackPIDF);
                shooterBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                shooterFront.setVelocity(shooterVel[0]);
                //shooterBack.setVelocity(shooterVel[2]);
                shooterBack.setPower(0.8);
                break;
            case "High":
                shooterFront.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, highShooterFrontPIDF);
                // shooterBack.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, highShooterBackPIDF);
                shooterBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shooterFront.setVelocity(shooterVel[1]);
                // shooterBack.setVelocity(shooterVel[3]);
                shooterBack.setPower(1.0);
                break;
            case "Idle": // idle vel
                  shooterFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                  shooterBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//                  shooterBack.setPower(0.35);
                  setPower(0.45, 0.35);
//                shooterFront.setVelocity(shooterVel[4]);
//                shooterBack.setVelocity(shooterVel[5]);
//                shooterFront.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, highShooterFrontPIDF);
//                shooterBack.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, highShooterBackPIDF);
                break;
        }
    }

    public void openBlocker() {
        blocker.setPosition(0);
    }

    public void closeBlocker() {
        blocker.setPosition(0.3);
    }

    public double getShooterFrontVel(){
        return (shooterFront.getVelocity()); // Returns in deg/sec
    }

    public double getShooterBackVel(){
        return (shooterBack.getVelocity()); // Returns in deg/sec
    }
    public boolean isAtHighVel() {
        //return true;
        return ((getShooterFrontVel() >= (shooterVel[1]-50)) && (getShooterBackVel() >= (shooterVel[3]-50)));
    }

    public boolean isAtLowVel() {
        //return true;
        return ((getShooterFrontVel() >= (shooterVel[0]-50)) && (getShooterBackVel() >= (shooterVel[2]-50)));
    }

    public DcMotorEx getShooterFrontMotor(){ return shooterFront;}
    public DcMotorEx getShooterBackMotor() { return shooterBack;}

    public PIDFCoefficients getHighShooterFrontPIDF(){ return highShooterFrontPIDF;}
    public PIDFCoefficients getHighShooterBackPIDF() { return highShooterBackPIDF;}
    public PIDFCoefficients getLowShooterFrontPIDF(){ return lowShooterFrontPIDF;}
    public PIDFCoefficients getLowShooterBackPIDF() { return lowShooterBackPIDF;}
}
