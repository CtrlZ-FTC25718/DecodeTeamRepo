package org.firstinspires.ftc.teamcode;

//Imports
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

//Hardware

public class Intake {

    private static DcMotor intakeMotor;

    private static Servo unJamServo;
    //State
    private double power;
    private boolean intakeOn;

    private int direction;

    //Constructor
    public Intake(HardwareMap map){
        intakeMotor = map.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        unJamServo = map.get(Servo.class, "slapServo");

        power = 0.0; //Default State: Not Running
        direction = 1; //Default direction is forward (intake mode)

    }

    public int getDirection(){
        return direction;
    }

    public void setDirection(int newDirection){ //Direction can be +1 or -1 to set direction
        direction = newDirection;
    }
    public boolean getIntakeState(){
        return intakeOn;
    }

    public void setIntakeState(boolean newState){
        intakeOn = newState;
    }
    public void setPower(double inputPower){
        power = inputPower;
    }
    public void releaseJam(){
        ElapsedTime jamTimer = new ElapsedTime();
        double jamReleaseTimer;
        for (int i = 0; i < 4; i++){
            jamReleaseTimer = jamTimer.milliseconds();
            this.setDirection(-1 * getDirection());
            this.update();
            while(jamTimer.milliseconds() - jamReleaseTimer < 300) {} //Do nothing
        }
        this.setIntakeState(false);
        this.update();
    }
    public void slapArtifact(){
      ElapsedTime slapTimer = new ElapsedTime();
      double slapReleaseTimer;
      unJamServo.setPosition(0.0);
      slapReleaseTimer = slapTimer.milliseconds();
      while(((slapTimer.milliseconds() - slapReleaseTimer) < 1500)){} //Do Nothing
      unJamServo.setPosition(0.31);
    }
    public void resetSlapper(){
        unJamServo.setPosition(0.31);
    }
    public double getPower(){
        return power;
    }

    public void update(){
        if(intakeOn){
            intakeMotor.setPower(direction * 1.0);

        }

        else{
            intakeMotor.setPower(direction * 0.0);
        }
    }

}
