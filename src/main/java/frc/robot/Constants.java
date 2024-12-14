// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }


  public static final class AprilTags{

        public static final double TAU = 2 * Math.PI;

            public static final String FRONT_RIGHT_CAMERA_NAME = "Mike's Little Buddy";
            public static final String FRONT_LEFT_CAMERA_NAME = "Brian's Little Buddy"; //Azul's_Little_Buddy
            public static final String BACK_RIGHT_CAMERA_NAME = "John's Little Buddy"; //Ben's_Little_Buddy (1)
            public static final String BACK_LEFT_CAMERA_NAME = "Roger's Little Buddy";

            /* For PhotonEstimator
            *             ^ 
            *             |
            *             Z        
            *      --------------
            *      |            |
            *      |            |
            *<-- X |     *Y     |
            *      |            |
            *      |            |
            *      --------------
            */

            // CHANGE THESE FOR OUR ROBOT
            public static final Transform3d ROBOT_TO_CAMERA_FRONT_LEFT = new Transform3d(Units.inchesToMeters(13.25)-0.04,Units.inchesToMeters(9.3), 0.25, new Rotation3d(0, -25./360*TAU, 20.*TAU/360));
            public static final Transform3d ROBOT_TO_CAMERA_FRONT_RIGHT = new Transform3d(Units.inchesToMeters(13.25)+0.04, Units.inchesToMeters(-9.3), 0.25, new Rotation3d(0, -25./360*TAU, -20.*TAU/360));
            public static final Transform3d ROBOT_TO_CAMERA_BACK_LEFT = new Transform3d(Units.inchesToMeters(-13.096)-0.03, Units.inchesToMeters(10.758)+0.03, 0.32, new Rotation3d(-15./360*TAU, -40./360*TAU, 32./360*TAU + TAU/2));
            public static final Transform3d ROBOT_TO_CAMERA_BACK_RIGHT = new Transform3d(Units.inchesToMeters(-13.096)+0.03, Units.inchesToMeters(-10.758)-0.03, 0.32, new Rotation3d(TAU/2 + 15./360*TAU, -40./360*TAU, -18./360*TAU-TAU/2));
            

            //With the Layout paths, REMEMBER you need to also upload the json file to the Photonvision GUI
            //This layout for some reason only works for the single tag estimation (as of 02/11/24) 
            public static final String LAYOUT_PATH = Filesystem.getDeployDirectory().getPath() + "/vision/2024-crescendo.json";

            public static final double getXSD(double distance) {
                  return 0.0312*distance - 0.0494;
            }

            public static final double getYSD(double distance) {
                  return 0.0656*distance - 0.129;
            }
      }

      public static final class Field {
            // public static final Translation2d SPEAKER_POSITION = new Translation2d(0, 0);
            public static final Translation3d BLUE_SPEAKER_POSITION = new Translation3d(-0.04, 5.9, 2.36); //y = 5.75
            public static final Translation3d BLUE_SPEAKER_POSITION_SOURCE = new Translation3d(-0.04, 6.15, 2.36); //y = 5.75  
            public static final Translation3d RED_SPEAKER_POSITION = new Translation3d(16.451, 5.45, 2.36); //y = 5.45
            public static final Translation3d RED_SPEAKER_POSITION_SOURCE = new Translation3d(16.451, 5.6, 2.36); //y = 5.45
            public static final Translation3d BLUE_STATION = new Translation3d(0, 4.48, 0);
            public static final Translation3d RED_STATION = new Translation3d(16.451, 4.48, 0);
            public static final Translation2d AMP_POSITION = new Translation2d(0, 0);
            public static final double LENGTH = 16.451;
            public static final double WIDTH = 8.211;

            public static final double CLOSE_FAR_CUTOFF = 3.15;
            public static final double SOURCE_AMP_CUTOFF = 4.1;

            //TODO: Check if this works!!
            public static final Translation3d getSpeakerPos(){
                  // double noteSpeed = 1;
                  // ChassisSpeeds fieldRelSpeeds = RobotContainer.drive.getFieldRelativeSpeeds();
                  // Pose2d robotPos = RobotContainer.drive.getPose();
                  // Translation3d speakerPos = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? BLUE_SPEAKER_POSITION : RED_SPEAKER_POSITION;
                  // double t = Math.abs((robotPos.getX()-speakerPos.getX())/(noteSpeed*RobotContainer.drive.getPose().getRotation().getCos()));
                  // double xOffset = fieldRelSpeeds.vxMetersPerSecond * t;
                  // double yOffset = fieldRelSpeeds.vyMetersPerSecond * t;
                  // Translation3d newSpeakerPos = new Translation3d(speakerPos.getX()-xOffset, speakerPos.getY()-yOffset, speakerPos.getZ());
                  if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                      if(RobotContainer.drive.getPose().getTranslation().getY() < SOURCE_AMP_CUTOFF) {
                        return BLUE_SPEAKER_POSITION_SOURCE;
                      } else {
                        return BLUE_SPEAKER_POSITION;
                        }
                    } else {
                            if(RobotContainer.drive.getPose().getTranslation().getY() < SOURCE_AMP_CUTOFF) {
                              return RED_SPEAKER_POSITION_SOURCE;
                            } else {
                              return RED_SPEAKER_POSITION;
                            }
                    }
            }

            public static final Translation3d getStation(){
                  return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? BLUE_STATION : RED_STATION;
            }
      }
}
