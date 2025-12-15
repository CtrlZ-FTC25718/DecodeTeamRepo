package org.firstinspires.ftc.teamcode;

//Imports
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {

    //Hardware
    private DcMotor shooterFront;
    private DcMotor shooterBack;
    private Servo blocker;

    //Constructor
    public Shooter(HardwareMap map){

        shooterFront = map.get(DcMotor.class, "shooterFrontMotor");
        shooterBack = map.get(DcMotor.class, "shooterBackMotor");
        blocker = map.get(Servo.class, "shooterBlocker");

        shooterFront.setDirection(DcMotor.Direction.REVERSE);
        shooterFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterBack.setDirection(DcMotor.Direction.REVERSE);
        shooterBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void setPower(double backPower, double frontPower){
        shooterFront.setPower(frontPower);
        shooterBack.setPower(backPower);
    }

    public void openBlocker(){
        blocker.setPosition(0);
    }

    public void restoreBlocker(){
        blocker.setPosition(0.3);
    }

}
