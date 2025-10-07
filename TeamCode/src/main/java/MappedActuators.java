import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MappedActuators {

//    private static final String taskServoOne_Name = "s0";
//    private static final String taskServoTwo_Name = "s1";

    private static final String intakeMotor_Name = "intakeMotor";

    private static final String shooterMotor_Name = "shooterMotor";

    private HardwareMap map;
//    public Servo taskServoOne;
//    public Servo taskServoTwo;
    private DcMotor intake;
    private DcMotor shooter;

    public MappedActuators(HardwareMap hardwareMap){
        map = hardwareMap;
//        taskServoOne = map.get(Servo.class, taskServoOne_Name);
//        taskServoTwo = map.get(Servo.class, taskServoTwo_Name);
        intake = map.get(DcMotor.class, intakeMotor_Name);
        shooter = map.get(DcMotor.class, shooterMotor_Name);

        // set intake properties
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set shooter properties
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

public void spinIntake(String direction){
    // Spin intake based upon direction
    if (direction.equals("intake")){
        // Intake
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setPower(1);
    }
    else if (direction.equals("reject")){
        // Reject
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setPower(1);
    }
    else {
        // Stop spinning
        intake.setPower(0);
    }
}

public void spinShooter(String action){
        // Start or stop the shooter
    if (action.equals("start")){
        shooter.setPower(.85);
    }

    if (action.equals("stop")){
        shooter.setPower(0);
    }
}


}
