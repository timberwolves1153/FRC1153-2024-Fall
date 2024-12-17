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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.vision.AprilTagManager;
import frc.robot.subsystems.vision.PhotonVisionSim;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.Wrist.Wrist;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (o+ther than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public static Drive drive;
  public static AprilTagManager aprilTags;
  public static PhotonVisionSim VisionSim;
  public static Elevator elevator;
  public static Wrist wrist;
  

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  //private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        
        drive = new Drive(
        new GyroIOPigeon2(),
        new ModuleIOTalonFX(0),
        new ModuleIOTalonFX(1),
        new ModuleIOTalonFX(2),
        new ModuleIOTalonFX(3));
        //aprilTags = new AprilTagManager();
        elevator = new Elevator(new ElevatorIOSparkMax());
        wrist = new Wrist();
        // flywheel = new Flywheel(new FlywheelIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        // aprilTags = new AprilTagManager();
        // VisionSim = new PhotonVisionSim();
        
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    //controller.start().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller
        .start()
        .onTrue(new InstantCommand(() -> drive.zeroAngle()));

      controller.a().onTrue(new InstantCommand(() -> elevator.setVoltage(-3)));
      controller.a().onFalse(new InstantCommand(() -> elevator.stop()));

      controller.y().onTrue(new InstantCommand(() -> elevator.setVoltage(3)));
      controller.y().onFalse(new InstantCommand(() -> elevator.stop()));  
      
      controller.x().onTrue(new InstantCommand(() -> wrist.setPivotVolts(1.5)));
      controller.x().onFalse(new InstantCommand(() -> wrist.setPivotVolts(0)));
      
      controller.b().onTrue(new InstantCommand(() -> wrist.setPivotVolts(-1.5)));
      controller.b().onFalse(new InstantCommand(() -> wrist.setPivotVolts(0)));
      
      controller.leftBumper().onTrue(new InstantCommand(() -> wrist.setRollerVolts(6)));
      controller.leftBumper().onFalse(new InstantCommand(() -> wrist.stopPivot()));

      controller.rightBumper().onTrue(new InstantCommand(() -> wrist.setRollerVolts(-6)));
      controller.rightBumper().onFalse(new InstantCommand(() -> wrist.stopPivot()));
            
    }

public Drive getDrive() {
      return drive;
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}