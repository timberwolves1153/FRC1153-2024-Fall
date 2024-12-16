package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonCamera;

import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;

public interface VisionIO{
    @AutoLog
    public static class VisionIOInputs {
      
        public Pose3d frontLeftPos = new Pose3d();
        public Pose2d frontLeftPose2d = new Pose2d();

        public double frontLeftTimeStamp = 0;    //ms 
        public boolean frontLeftHasTarget = false;
        public int leftTagsSeen = 0;
        public double leftAmbiguity = 0;    //ratio for ambiguity

        public double leftYaw = 0;  //rad

        public Pose3d rightPos = new Pose3d();
        public Pose2d rightPose2d = new Pose2d();

        public double rightTimeStamp = 0;    //ms 
        public boolean rightHasTarget = false;
        public int rightTagsSeen = 0;
        public double rightAmbiguity = 0;    //ratio for ambiguity

        public double rightYaw = 0;  //rad

        double leftDistToSpeaker = 0;
        double rightDistToSpeaker = 0;

        public Pose3d backLeftPos = new Pose3d();
        public Pose2d backLeftPose2d = new Pose2d();
        public double backLeftTimeStamp = 0;   //ms
        public boolean backLeftHasTarget = false;
        public double backLeftAmbiguity = 0;

        public Pose3d backRightPos = new Pose3d();
        public Pose2d backRightPose2d = new Pose2d();
        public double backRightTimeStamp = 0;   //ms
        public boolean backRightHasTarget = false;
        public double backRightAmbiguity = 0;

        public PhotonCamera frontRightCamera = new PhotonCamera(Constants.AprilTags.FRONT_RIGHT_CAMERA_NAME);   //Right
        public PhotonCamera frontLeftCamera = new PhotonCamera(Constants.AprilTags.FRONT_LEFT_CAMERA_NAME);   //Left
        public PhotonCamera backLeftCamera = new PhotonCamera(Constants.AprilTags.BACK_LEFT_CAMERA_NAME);
        public PhotonCamera backRightCamera = new PhotonCamera(Constants.AprilTags.BACK_RIGHT_CAMERA_NAME);
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}