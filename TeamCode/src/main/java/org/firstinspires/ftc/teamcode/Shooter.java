package org.firstinspires.ftc.teamcode;

//Imports
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private PIDFCoefficients shooterFrontPIDF;
    private PIDFCoefficients shooterBackPIDF;
    private final double[] shooterVel = {1500, 2000, 1500, 2000}; // In Deg/Sec: Front Low, Front High, Back Low, Back High

    //Constructor
    public Shooter(HardwareMap map) {

        shooterFront = map.get(DcMotorEx.class, "shooterFrontMotor");
        shooterBack = map.get(DcMotorEx.class, "shooterBackMotor");
        blocker = map.get(Servo.class, "shooterBlocker");

        shooterFront.setDirection(DcMotorEx.Direction.REVERSE);
        shooterFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterBack.setDirection(DcMotorEx.Direction.REVERSE);
        shooterBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterFrontPIDF = new PIDFCoefficients(30, 0, 0, 13.3);
        shooterBackPIDF = new PIDFCoefficients(23, 0, 0, 13);

        shooterFront.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterFrontPIDF);
        shooterBack.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterBackPIDF);
    }

    public void setPower(double backPower, double frontPower) {
        shooterFront.setPower(frontPower);
        shooterBack.setPower(backPower);
    }

    public void setVelocity(String level) {
        if (level.equals("Low")) {
            shooterFront.setVelocity(shooterVel[0]);
            shooterBack.setVelocity(shooterVel[2]);
        } else {
            shooterFront.setVelocity(shooterVel[1]);
            shooterBack.setVelocity(shooterVel[3]);
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
        return ((getShooterFrontVel() >= shooterVel[1]) && (getShooterBackVel() >= shooterVel[3]));
    }

    public boolean isAtLowVel() {
        return ((getShooterFrontVel() >= shooterVel[0]) && (getShooterBackVel() >= shooterVel[2]));
    }

    public DcMotorEx getShooterFrontMotor(){ return shooterFront;}
    public DcMotorEx getShooterBackMotor() { return shooterBack;}

    public PIDFCoefficients getShooterFrontPIDF(){ return shooterFrontPIDF;}
    public PIDFCoefficients getShooterBackPIDF() { return shooterBackPIDF;}
}
