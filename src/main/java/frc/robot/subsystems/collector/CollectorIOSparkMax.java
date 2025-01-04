package frc.robot.subsystems.collector;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class CollectorIOSparkMax implements CollectorIO{

    private CANSparkMax collectorMotor;
    
    public CollectorIOSparkMax() {

        collectorMotor = new CANSparkMax(53, MotorType.kBrushless);

        configMotors();

    }

    public void configMotors() {
        collectorMotor.restoreFactoryDefaults();
        collectorMotor.clearFaults();
        collectorMotor.setIdleMode(IdleMode.kBrake);
        collectorMotor.setInverted(false);
        collectorMotor.setSmartCurrentLimit(40);
        collectorMotor.burnFlash();
    }

    @Override
    public void updateInputs(CollectorIOInputs inputs) {
        inputs.appliedVolts = collectorMotor.getAppliedOutput() * 12;
        inputs.currentAmps = collectorMotor.getOutputCurrent();
    }


    @Override
    public void setVoltage(double volts) {
        collectorMotor.setVoltage(volts);
    }

    @Override
    public void stop() {
        collectorMotor.setVoltage(0);
    }
}
