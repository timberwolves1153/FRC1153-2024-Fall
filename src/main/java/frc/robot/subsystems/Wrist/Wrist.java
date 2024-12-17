package frc.robot.subsystems.Wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase{

    private CANSparkMax pivot, roller;
    
    public Wrist() {

        pivot = new CANSparkMax(51, MotorType.kBrushless);
        roller = new CANSparkMax(52, MotorType.kBrushless);

    }

    public void configMotors() {
        pivot.restoreFactoryDefaults();
        roller.restoreFactoryDefaults();
        pivot.clearFaults();
        pivot.setIdleMode(IdleMode.kBrake);
        pivot.setInverted(false);
        pivot.setSmartCurrentLimit(40);


        
        roller.clearFaults();
        roller.setIdleMode(IdleMode.kBrake);
        roller.setSmartCurrentLimit(40);
        pivot.burnFlash();
        roller.burnFlash();

    }

    public void setRollerVolts(double volts) {
        roller.setVoltage(volts);
    }

    public void stopRollers() {
        roller.setVoltage(0);
    }
    public void setPivotVolts(double volts) {
        pivot.setVoltage(volts);
    }
    public void stopPivot() {
        roller.setVoltage(0);
    }

}