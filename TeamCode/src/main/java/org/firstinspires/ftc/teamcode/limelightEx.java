package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

public class limelightEx {

    private Limelight3A limelight;
    private LLResult result;

    public limelightEx(HardwareMap map){

        limelight = map.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        limelight.start();

        result = limelight.getLatestResult();
    }

    //Recommend filterSize = 30;
    public double[] buffered_localize(int filterSize){
        double[] posEst = new double[]{0, 0, 0};
        for(int i = 0; i < filterSize; i++){
            double[] currentEst = this.localize();
            posEst[0] += currentEst[0];
            posEst[1] += currentEst[1];
            posEst[2] += currentEst[2];
        }
        posEst[0] = posEst[0] / filterSize;
        posEst[1] = posEst[1] / filterSize;
        posEst[2] = posEst[2] / filterSize;
        return posEst;
    }

    private double[] localize(){

        result = limelight.getLatestResult();

        if (result.isValid()) {
            Pose3D botpose = result.getBotpose();
            Position coords = botpose.getPosition();
            coords.toUnit(DistanceUnit.INCH);
            return this.transform_frame(coords, botpose);
        }
        else{
            return new double[]{0, 0, 0};
        }
    }

    private double[] transform_frame(Position coords, Pose3D botpose){
        double x;
        double y;
        double theta;

        x = coords.y + 72;
        y = -coords.x + 72;
        theta = botpose.getOrientation().getYaw(AngleUnit.DEGREES) + 270;
        theta = theta % 360;

        return new double[]{x, y, theta};
    }

}
