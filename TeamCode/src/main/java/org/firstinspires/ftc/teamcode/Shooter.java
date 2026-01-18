package org.firstinspires.ftc.teamcode;

//Imports
import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

public class Shooter {

    //Hardware
    private DcMotorEx shooterFront;
    private DcMotorEx shooterBack;
    private Servo blocker;
    private PIDFCoefficients shooterBackPIDF, shooterFrontPIDF;

    private VoltageSensor voltmeter;

    private final double idealVoltage = 13.1; // Full charge voltage on the battery

    // In Tics/Sec: Front Low, Front High, Back Low, Back High, idle front, idle back, custom front, custom back

    private final double[] shooterVel = {1350, 1600, 1350, 1550, 0, 0, 0, 0};
    private final double[] targetPos = {140, 140, 0, 140}; // Holds RedTargetX, RedTargetY, BlueTargetX, BlueTargetY

    private double robotEnergy;

    //Constructor
    public Shooter(HardwareMap map) {

        shooterFront = map.get(DcMotorEx.class, "shooterFrontMotor");
        shooterBack = map.get(DcMotorEx.class, "shooterBackMotor");
        blocker = map.get(Servo.class, "shooterBlocker");

        shooterFront.setDirection(DcMotorEx.Direction.REVERSE);
        shooterFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterBack.setDirection(DcMotorEx.Direction.FORWARD);
        shooterBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterFrontPIDF =  new PIDFCoefficients(60, 0, 0, 27);
        shooterBackPIDF = shooterFrontPIDF;
        //shooterFrontPIDF = new PIDFCoefficients(50, 0,0, 22.5); // 50, 0, 0, 20.4
        //shooterBackPIDF = new PIDFCoefficients(800, 70, 70, 20);
        //shooterBackPIDF =  new PIDFCoefficients(55,0,0,27.5); // 55, 0, 0, 25

        shooterFront.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterFrontPIDF);
        shooterBack.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterBackPIDF);

        voltmeter =  map.voltageSensor.iterator().next();
    }

    public void computeEnergy(){
        robotEnergy = Math.pow(idealVoltage/voltmeter.getVoltage(), 1); // robotEnergy is a ratio
        //Log.d("Voltage Ratio", "Robot Energy" + robotEnergy);
    }

    public void setPower(double backPower, double frontPower) {
        shooterFront.setPower(frontPower);
        shooterBack.setPower(backPower);
    }

    // This function accepts the robot point and shooter target point and outputs shooter angular velocity in tps and the heading the robot should turn to
    public double[] computeCustomShotParameters(double global_x, double global_y, String teamColor) {

        double target_x = 0, target_y = 0;

        // Determine target_x and target_y based upon teamColor
        switch(teamColor){
            case "Red":
                target_x = targetPos[0];
                target_y = targetPos[1];
                break;

            case "Blue":
                target_x = targetPos[2];
                target_y = targetPos[3];
                break;
        }

        //Compute distance to target
        double delta = Math.sqrt(Math.pow((target_x - global_x), 2) + Math.pow((target_y - global_y), 2)) / 12; // in ft

        //Compute launch angle
        double theta = Math.toRadians(55); // This 55 deg is fixed for our robot based upon shooter positioning

        //Compute exit velocity magnitude
        double exitVel = Math.sqrt((-16 * Math.pow(delta, 2)) / ((3.5 - delta * Math.tan(theta) - 0.5) * (Math.pow(Math.cos(theta), 2))));

        //Compute shooter target speed
        double realOmega = (2 / 0.23622) * (28 / (2 * Math.PI)) * (1 / 0.52) * exitVel; // shot energy transfer efficiency is an estimate of 45%

        //Compute the angle the robot should turn to
        double psi = Math.toDegrees(Math.atan2(target_y - global_y, target_x - global_x));

        //Compute Offset Pose:
        double offsetAngle = Math.toDegrees(Math.atan(9/5)) + psi;
        double xOffsetPos = Math.sqrt(Math.pow(9, 2) + Math.pow(5,2)) * Math.cos(Math.toRadians(offsetAngle)) + global_x;
        double yOffsetPos = Math.sqrt(Math.pow(9, 2) + Math.pow(5,2)) * Math.sin(Math.toRadians(offsetAngle)) + global_y;
        //Output results
        return new double[]{realOmega, psi, xOffsetPos, yOffsetPos}; // in tics/sec and deg
    }

    // Use velocityHold for initial spin-up of shooter
    public void velocityHold(String level, double rate) {

        // Not using PIDs for velocity hold as it is brute force and fast
        shooterBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Giving 50 Tics/sec tollerance

        switch (level) {
            case "High":
                //while (shooterBack.getVelocity() < (shooterVel[3]-50) && shooterFront.getVelocity() < (shooterVel[1]-50) && shooterFront.getPower() != 1 && shooterBack.getPower() != 1) {
                while((shooterBack.getVelocity() + shooterFront.getVelocity())/2 < (shooterVel[3]) && shooterFront.getPower() != 1 && shooterBack.getPower() != 1){
                    shooterBack.setPower(robotEnergy * (shooterBack.getPower() + rate));
                    shooterFront.setPower(robotEnergy * (shooterFront.getPower() + rate));
//                    Log.d("Velocity Hold Target","High Velocity Hold - Waiting for Front to be " + shooterVel[1]);
//                    Log.d("Velocity Hold Target","High Velocity Hold - Waiting for Back to be " + shooterVel[3]);
                }
                break;

            case "Low":
                //while (shooterBack.getVelocity() < (shooterVel[2]-50) && shooterFront.getVelocity() < (shooterVel[0]-50) && shooterFront.getPower() != 1 && shooterBack.getPower() != 1) {
                while((shooterBack.getVelocity() + shooterFront.getVelocity())/2 < (shooterVel[0] - 50) && shooterFront.getPower() != 1 && shooterBack.getPower() != 1){
                    shooterBack.setPower(robotEnergy * (shooterBack.getPower() + rate));
                    shooterFront.setPower(robotEnergy * (shooterFront.getPower() + rate));
                }
                break;
            case "Custom":
                //while (shooterBack.getVelocity() < (shooterVel[7]-50) && shooterFront.getVelocity() < (shooterVel[6]-50) && shooterFront.getPower() != 1 && shooterBack.getPower() != 1) {
                while((shooterBack.getVelocity() + shooterFront.getVelocity())/2 < (shooterVel[6] - 50) && shooterFront.getPower() != 1 && shooterBack.getPower() != 1){
                    shooterBack.setPower(robotEnergy * (shooterBack.getPower() + rate));
                    shooterFront.setPower(robotEnergy * (shooterFront.getPower() + rate));
                }
                break;
        }

        shooterFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setVelocity(String level) {
        switch (level) {
            case "Low":
                // Use PID for Low velocity shots (close shots)
                shooterFront.setVelocity(shooterVel[0]);
                shooterBack.setVelocity(shooterVel[2]);
                break;

            case "High":

                shooterBack.setVelocity(shooterVel[3]);
                shooterFront.setVelocity(shooterVel[1]);
                //shooterBack.setPower(1.0);
                //shooterFront.setPower(1.0);
                break;

            case "Idle": // idle vel
                shooterBack.setVelocity(shooterVel[5]);
                shooterFront.setVelocity(shooterVel[4]);
                break;

            case "Custom": // customVel
                shooterBack.setVelocity(shooterVel[7]);
                shooterFront.setVelocity(shooterVel[6]);
        }
    }

    public void updateCustomVelocity(double customBackVel, double customFrontVel){
        shooterVel[6] = customFrontVel;
        shooterVel[7] = customBackVel;
    }

    public double[] getCustomVelocities(){
        return new double[]{shooterVel[6], shooterVel[7]};
    }

    public void openBlocker() {
        blocker.setPosition(0);
    }

    public void closeBlocker() {
        blocker.setPosition(0.4);
    }

    public double getShooterFrontVel() {
        return (shooterFront.getVelocity()); // Returns in deg/sec
    }

    public double getShooterBackVel() {
        return (shooterBack.getVelocity()); // Returns in deg/sec
    }

    public boolean isAtHighVel() {
        //return true;
       // return ((getShooterFrontVel() >= (shooterVel[1] - 50)) && (getShooterBackVel() >= (shooterVel[3] - 50)));
        return ((getShooterFrontVel() + getShooterBackVel())/2 >= (shooterVel[1]));
    }

    public boolean isAtLowVel() {
        //return true;
        //return ((getShooterFrontVel() >= (shooterVel[0] - 50)) && (getShooterBackVel() >= (shooterVel[2] - 50)));
        return ((getShooterFrontVel() + getShooterBackVel())/2 >= (shooterVel[0] - 50));
    }

    public boolean isAtCustomVel() {
        //return ((getShooterFrontVel() >= (shooterVel[6] - 50)) && (getShooterBackVel() >= (shooterVel[7] - 50)));
        return ((getShooterFrontVel() + getShooterBackVel())/2 >= (shooterVel[6] - 50));
    }

    public DcMotorEx getShooterFrontMotor() {
        return shooterFront;
    }

    public DcMotorEx getShooterBackMotor() {
        return shooterBack;
    }

    public PIDFCoefficients getShooterFrontPIDF() {
        return shooterFrontPIDF;
    }

    public PIDFCoefficients getShooterBackPIDF() {
        return shooterBackPIDF;
    }
}
