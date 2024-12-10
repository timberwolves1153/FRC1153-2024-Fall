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

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.vision.AprilTagManager;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    Pose2d leftPose2d = AprilTagManager.getLeftRobotPos().toPose2d();
    Pose2d rightPose2d = AprilTagManager.getRightRobotPos().toPose2d();

    double leftError = leftPose2d.relativeTo(RobotContainer.drive.getPose()).getTranslation().getNorm();
    double rightError = rightPose2d.relativeTo(RobotContainer.drive.getPose()).getTranslation().getNorm();

    Pose2d backLeftPose2d = AprilTagManager.getBackLeftPos().toPose2d();
    Pose2d backRightPose2d = AprilTagManager.getBackRightPos().toPose2d();

    double backLeftError = backLeftPose2d.relativeTo(RobotContainer.drive.getPose()).getTranslation().getNorm();
    double backRightError = backRightPose2d.relativeTo(RobotContainer.drive.getPose()).getTranslation().getNorm();

    Logger.recordOutput("LeftErrorDist", leftError);
    Logger.recordOutput("rightErrorDist", rightError);

      if(DriverStation.isAutonomous()) {
        if(AprilTagManager.hasLeftTarget()
            && AprilTagManager.getLeftAmbiguity() <= 0.15
            && AprilTagManager.getLeftRobotPos() != null
            && leftError < 1
            && leftPose2d.getX() > 0 && leftPose2d.getX() < Constants.Field.LENGTH
            && leftPose2d.getY() > 0 && leftPose2d.getY() < Constants.Field.WIDTH)
              RobotContainer.drive.updateOdometryWithVision(leftPose2d, AprilTagManager.getLeftTimestamp());

        if(AprilTagManager.hasRightTarget()
            && AprilTagManager.getRightAmbiguity() <= 0.15
            && AprilTagManager.getRightRobotPos() != null
            && rightError < 1
            && rightPose2d.getX() > 0 && rightPose2d.getX() < Constants.Field.LENGTH
            && rightPose2d.getY() > 0 && rightPose2d.getY() < Constants.Field.WIDTH)
              RobotContainer.drive.updateOdometryWithVision(rightPose2d, AprilTagManager.getRightTimestamp());

              if(AprilTagManager.hasBackLeftTarget()
            && AprilTagManager.getBackLeftAmbiguity() <= 0.15
            && AprilTagManager.getBackLeftPos() != null
            && backLeftError < 1
            && backLeftPose2d.getX() > 0 && backLeftPose2d.getX() < Constants.Field.LENGTH
            && backLeftPose2d.getY() > 0 && backLeftPose2d.getY() < Constants.Field.WIDTH)
              RobotContainer.drive.updateOdometryWithVision(backLeftPose2d, AprilTagManager.getBackLeftTimestamp());

              if(AprilTagManager.hasBackRightTarget()
            && AprilTagManager.getBackRightAmbiguity() <= 0.15
            && AprilTagManager.getBackRightPos() != null
            && backRightError < 1
            && backRightPose2d.getX() > 0 && backRightPose2d.getX() < Constants.Field.LENGTH
            && backRightPose2d.getY() > 0 && backRightPose2d.getY() < Constants.Field.WIDTH)
              RobotContainer.drive.updateOdometryWithVision(backRightPose2d, AprilTagManager.getBackRightTimestamp());
      } else if (!RobotContainer.drive.overrideVisionOdo){
        if(AprilTagManager.hasLeftTarget()
            && AprilTagManager.getLeftAmbiguity() <= 0.15
            && AprilTagManager.getLeftRobotPos() != null
            && leftError < 5
            && leftPose2d.getX() > 0 && leftPose2d.getX() < Constants.Field.LENGTH
            && leftPose2d.getY() > 0 && leftPose2d.getY() < Constants.Field.WIDTH)
              RobotContainer.drive.updateOdometryWithVision(leftPose2d, AprilTagManager.getLeftTimestamp());
        if(AprilTagManager.hasRightTarget()
            && AprilTagManager.getRightAmbiguity() <= 0.15
            && AprilTagManager.getRightRobotPos() != null
            && rightError < 5
            && rightPose2d.getX() > 0 && rightPose2d.getX() < Constants.Field.LENGTH
            && rightPose2d.getY() > 0 && rightPose2d.getY() < Constants.Field.WIDTH)
              RobotContainer.drive.updateOdometryWithVision(rightPose2d, AprilTagManager.getRightTimestamp());

        if(AprilTagManager.hasBackLeftTarget()
            && AprilTagManager.getBackLeftAmbiguity() <= 0.15
            && AprilTagManager.getBackLeftPos() != null
            && backLeftError < 2
            && backLeftPose2d.getX() > 0 && backLeftPose2d.getX() < Constants.Field.LENGTH
            && backLeftPose2d.getY() > 0 && backLeftPose2d.getY() < Constants.Field.WIDTH)
              RobotContainer.drive.updateOdometryWithVision(backLeftPose2d, AprilTagManager.getBackLeftTimestamp());

              if(AprilTagManager.hasBackRightTarget()
            && AprilTagManager.getBackRightAmbiguity() <= 0.15
            && AprilTagManager.getBackRightPos() != null
            && backRightError < 2
            && backRightPose2d.getX() > 0 && backRightPose2d.getX() < Constants.Field.LENGTH
            && backRightPose2d.getY() > 0 && backRightPose2d.getY() < Constants.Field.WIDTH)
              RobotContainer.drive.updateOdometryWithVision(backRightPose2d, AprilTagManager.getBackRightTimestamp());
      }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}