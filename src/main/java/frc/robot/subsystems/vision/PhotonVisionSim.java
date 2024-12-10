// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.AprilTags;

import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import com.google.flatbuffers.Constants;

/** IO implementation for physics sim using PhotonVision simulator. */
public class PhotonVisionSim extends PhotonVisionIO {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim frontLeftCameraSim;
  private final PhotonCameraSim frontRightCameraSim;
  private final PhotonCameraSim backLeftCameraSim;
  private final PhotonCameraSim backRightCameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public PhotonVisionSim(Supplier<Pose2d> poseSupplier) {
    
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(layout);
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    frontLeftCameraSim = new PhotonCameraSim(frontLeftCamera, cameraProperties);
    visionSim.addCamera(frontLeftCameraSim, AprilTags.ROBOT_TO_CAMERA_FRONT_LEFT);
  
    frontRightCameraSim = new PhotonCameraSim(frontRightCamera, cameraProperties);
    visionSim.addCamera(frontRightCameraSim, AprilTags.ROBOT_TO_CAMERA_FRONT_RIGHT);

    backLeftCameraSim = new PhotonCameraSim(backLeftCamera, cameraProperties);
    visionSim.addCamera(frontLeftCameraSim, AprilTags.ROBOT_TO_CAMERA_BACK_LEFT);

    backRightCameraSim = new PhotonCameraSim(backRightCamera, cameraProperties);
    visionSim.addCamera(frontLeftCameraSim, AprilTags.ROBOT_TO_CAMERA_BACK_RIGHT);

  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}