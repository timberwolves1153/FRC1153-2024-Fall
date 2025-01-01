package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class WristIOSparkMax implements WristIO{

    private CANSparkMax pivotMotor;
    private CANcoder encoder;
    
    public WristIOSparkMax() {

        pivotMotor = new CANSparkMax(51, MotorType.kBrushless);
        encoder = new CANcoder(52);

        configMotors();
    }

    @Override
    public void updateInputs(WristInputs inputs) {
        inputs.absolutePosition = Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble());
        inputs.absolutePositionRadians = encoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        inputs.appliedVolts = pivotMotor.getAppliedOutput() * 12;
    }

    public void configMotors() {
        pivotMotor.restoreFactoryDefaults();
        pivotMotor.clearFaults();
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setInverted(false);
        pivotMotor.setSmartCurrentLimit(40);
        pivotMotor.burnFlash();
    }

    @Override
    public void setVoltage(double volts) {
        pivotMotor.setVoltage(volts);
    }

    @Override
    public void stop() {
        pivotMotor.setVoltage(0);
    }

    public double getAbsolutePosition() {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getPositionRadians() {
        return Units.rotationsToRadians(getAbsolutePosition());
    }

    public double getPositionDegrees() {
        return Units.rotationsToDegrees(getAbsolutePosition());
    }
}
