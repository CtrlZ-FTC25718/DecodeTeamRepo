package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MappedActuators {

    private static final String taskServoOne_Name = "s0";
    private static final String taskServoTwo_Name = "s1";

    private HardwareMap map;
    public Servo taskServoOne;
    public Servo taskServoTwo;

    public MappedActuators(HardwareMap hardwareMap){
        map = hardwareMap;
        taskServoOne = map.get(Servo.class, taskServoOne_Name);
        taskServoTwo = map.get(Servo.class, taskServoTwo_Name);
    }
}
