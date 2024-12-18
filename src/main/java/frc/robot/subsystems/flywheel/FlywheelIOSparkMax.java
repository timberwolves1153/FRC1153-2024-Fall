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

package frc.robot.subsystems.flywheel;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class FlywheelIOSparkMax implements FlywheelIO {
  private static final double GEAR_RATIO = 1 / 112; // NEED CONFIG is it 1/112

  private final CANSparkMax wristMotor = new CANSparkMax(51, MotorType.kBrushless);
  private final RelativeEncoder encoder = wristMotor.getEncoder(); // Check for type of encoder
  private PIDController pid = new PIDController(0.0, 0.0, 0.0); // CHECK FOR THING.

  public FlywheelIOSparkMax() {
    wristMotor.restoreFactoryDefaults();

    wristMotor.setInverted(false); // CHEKC FOR FUTURE REFERENCE

    wristMotor.enableVoltageCompensation(12.0);

    wristMotor.setSmartCurrentLimit(40);

    wristMotor.setIdleMode(IdleMode.kBrake);

    wristMotor.burnFlash();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
    inputs.currentAmps = new double[] {wristMotor.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    wristMotor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void stop() {
    wristMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }

  
  public double getWristDegrees(){
    return encoder.getPosition();
  }
}
