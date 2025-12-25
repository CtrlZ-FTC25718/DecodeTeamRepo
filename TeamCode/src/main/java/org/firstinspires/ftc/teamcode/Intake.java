package org.firstinspires.ftc.teamcode;

//Imports
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;

//Hardware

public class Intake {

    private static DcMotor intakeMotor;
    //State
    private double power;
    private boolean intakeOn;

    //Constructor
    public Intake(HardwareMap map){
        intakeMotor = map.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        power = 0.0; //Default State: Not Running

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

    public double getPower(){
        return power;
    }

    public void update(){
        if(intakeOn){
            intakeMotor.setPower(1.0);
        }

        else{
            intakeMotor.setPower(0.0);
        }
    }

}
