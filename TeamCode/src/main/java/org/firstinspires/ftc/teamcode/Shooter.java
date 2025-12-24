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
    private PIDFCoefficients shooterBackPIDF, shooterFrontPIDF;
    private final double[] shooterVel = {1500, 1750, 1500, 1500, 0, 0}; // In Deg/Sec: Front Low, Front High, Back Low, Back High, idle front, idle back

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

        shooterFrontPIDF = new PIDFCoefficients(50, 0,0, 16.4);
        shooterBackPIDF = new PIDFCoefficients(800, 70, 70, 20);

        shooterFront.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterFrontPIDF);
        shooterBack.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterBackPIDF);
    }

    public void setPower(double backPower, double frontPower) {
        shooterFront.setPower(frontPower);
        shooterBack.setPower(backPower);
    }

    // This function accepts the robot point and shooter target point and outputs shooter angular velocity in tps and the heading the robot should turn to
    public double[] computeProjectileMotion(double global_x, double global_y, double target_x, double target_y) {

        //Compute distance to target
        double delta = Math.sqrt(Math.pow((target_x - global_x), 2) + Math.pow((target_y - global_y), 2));

        //Compute launch angle
        double theta = Math.toRadians(58);

        //Compute exit velocity magnitude
        double exitVel = Math.sqrt((-16 * Math.pow(delta, 2)) / ((3.5 - delta * Math.tan(theta) - 0.5) * (Math.pow(Math.cos(theta), 2))));

        //Compute shooter target speed
        double realOmega = (2 / 0.23622) * (28 / (2 * Math.PI)) * (1 / .405) * exitVel;

        //Compute the angle the robot should turn to
        double psi = Math.toDegrees(Math.atan2(target_y - global_y, target_x - global_x));

        //Output results
        return new double[]{realOmega, psi};
    }

    // Use velocityHold for initial spin-up of shooter
    public void velocityHold(String level, double rate, double customBackVel, double customFrontVel) {

        // Not using PIDs for velocity hold as it is brute force and fast
        shooterBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        switch (level) {
            case "High":
                while (shooterBack.getVelocity() < shooterVel[3] && shooterFront.getVelocity() < shooterVel[1] && shooterFront.getPower() != 1 && shooterBack.getPower() != 1) {
                    shooterBack.setPower(shooterBack.getPower() + rate);
                    shooterFront.setPower(shooterFront.getPower() + rate);
                }
                break;

            case "Low":
                while (shooterBack.getVelocity() < shooterVel[2] && shooterFront.getVelocity() < shooterVel[0] && shooterFront.getPower() != 1 && shooterBack.getPower() != 1) {
                    shooterBack.setPower(shooterBack.getPower() + rate);
                    shooterFront.setPower(shooterFront.getPower() + rate);
                }
                break;

            case "Custom":
                while (shooterBack.getVelocity() < customBackVel && shooterFront.getVelocity() < customFrontVel && shooterFront.getPower() != 1 && shooterBack.getPower() != 1) {
                    shooterBack.setPower(shooterBack.getPower() + rate);
                    shooterFront.setPower(shooterFront.getPower() + rate);
                }
                break;
        }

        shooterFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setVelocity(String level, double customBackVel, double customFrontVel) {
        switch (level) {
            // Ignore customBackVel and customFrontVel for Low, High, and Idle
            case "Low":
                // Use PID for Low velocity shots (close shots)
//                shooterBack.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterBackPIDF);
//                shooterFront.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterFrontPIDF);
                shooterFront.setVelocity(shooterVel[0]);
                shooterBack.setVelocity(shooterVel[2]);
                break;

            case "High":
                // PID will fall short for High velocity shots - go max power (long shots)
//                shooterBack.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterBackPIDF);
//                shooterFront.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterFrontPIDF);
                shooterBack.setPower(1.0);
                shooterFront.setPower(1.0);
                break;

            case "Idle": // idle vel
//                shooterBack.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterBackPIDF);
//                shooterFront.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterFrontPIDF);
                shooterBack.setVelocity(shooterVel[4]);
                shooterFront.setVelocity(shooterVel[5]);
                break;

            case "Custom": // Use custom Velocity
//                shooterBack.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterBackPIDF);
//                shooterFront.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterFrontPIDF);
                shooterBack.setVelocity(customBackVel);
                shooterFront.setVelocity(customFrontVel);
        }
    }

    public void openBlocker() {
        blocker.setPosition(0);
    }

    public void closeBlocker() {
        blocker.setPosition(0.3);
    }

    public double getShooterFrontVel() {
        return (shooterFront.getVelocity()); // Returns in deg/sec
    }

    public double getShooterBackVel() {
        return (shooterBack.getVelocity()); // Returns in deg/sec
    }

    public boolean isAtHighVel() {
        //return true;
        // Give 50 tics/sec wiggle room
        return ((getShooterFrontVel() >= (shooterVel[1] - 50)) && (getShooterBackVel() >= (shooterVel[3] - 50)));
    }

    public boolean isAtLowVel() {
        //return true;
        // Give 50 tics/sec wiggle room
        return ((getShooterFrontVel() >= (shooterVel[0] - 50)) && (getShooterBackVel() >= (shooterVel[2] - 50)));
    }

    public boolean isAtCustomVel(double customBackVel, double customFrontVel) {
        return ((getShooterFrontVel() >= customFrontVel) && (getShooterBackVel() >= customBackVel));
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
