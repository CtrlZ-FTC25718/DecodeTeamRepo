package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class limelightDetector {

    static private WebcamName camera;
    static private double[] cameraOffsetPose;
    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    public limelightDetector(HardwareMap map){

        camera = map.get(WebcamName.class, "limelight");
        cameraOffsetPose = new double[]{4.5, 2.5, 0};
        initAprilTag();

    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())

                .setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(1196.94, 1196.94, 804.849, 411.195)
                // ... these parameters are fx, fy, cx, cy.


                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera
        builder.setCamera(camera);


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1600, 896));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()*/

    public double[] visualLocalization(double[] aprilTagPosition, int aprilTagID, int filterSize){

        aprilTagPosition[2] *= Math.PI / 180;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double xDist = 0;
        double yDist = 0;
        double thetaDiff = 0;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == aprilTagID) {
                double xSum = 0;
                double ySum = 0;
                double thetaSum = 0;
                for(int i = 0; i < filterSize; i++){
                    xSum += detection.ftcPose.x;
                    ySum += detection.ftcPose.y;
                    thetaSum += detection.ftcPose.yaw;
                }
                xDist = xSum / filterSize;
                yDist = ySum / filterSize;
                thetaDiff = thetaSum / filterSize;
            }
        }   // end for() loop

        double thetaPos = aprilTagPosition[2] - thetaDiff + Math.PI;

        double xPos = aprilTagPosition[0] + (Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2)) * Math.cos(thetaPos - (Math.PI / 2)));
        double yPos = aprilTagPosition[1] + (Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2)) * Math.sin(thetaPos - (Math.PI / 2)));
        thetaPos -= cameraOffsetPose[2];
        double beta = 0;
        if(cameraOffsetPose[0] > 0){
            beta = Math.atan(cameraOffsetPose[1] / cameraOffsetPose[0]);
        }
        else if(cameraOffsetPose[0] < 0){
            beta = Math.atan(cameraOffsetPose[1]) + Math.PI;
        }
        else if(cameraOffsetPose[0] == 0 && cameraOffsetPose[1] > 0){
            beta = Math.PI / 2;
        }
        else{
            beta = - Math.PI / 2;
        }
        xPos += (Math.sqrt(Math.pow(cameraOffsetPose[0], 2) + Math.pow(cameraOffsetPose[1], 2)) * Math.cos(thetaPos + beta - Math.PI));
        yPos += (Math.sqrt(Math.pow(cameraOffsetPose[0], 2) + Math.pow(cameraOffsetPose[1], 2)) * Math.sin(thetaPos + beta - Math.PI));

        return new double[]{xPos, yPos, thetaPos * 180 / Math.PI};

    }




}
