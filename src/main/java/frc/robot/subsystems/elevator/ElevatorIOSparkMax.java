package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ElevatorIOSparkMax implements ElevatorIO{

    private CANSparkMax leftMotor, rightMotor;
    private RelativeEncoder encoder;
    
    public ElevatorIOSparkMax() {

        leftMotor = new CANSparkMax(41, MotorType.kBrushless);
        rightMotor = new CANSparkMax(42, MotorType.kBrushless);
        encoder = leftMotor.getEncoder();

       

        configMotors();

    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.positionInches = ticksToInches();
    }


     public void configMotors() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        leftMotor.clearFaults();
        leftMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.setInverted(false);
        leftMotor.setSmartCurrentLimit(40);


        
        rightMotor.clearFaults();
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.follow(leftMotor, false);
        rightMotor.setSmartCurrentLimit(40);
        rightMotor.burnFlash();
        leftMotor.burnFlash();
    }

    @Override
    public void setVoltage(double volts) {
        leftMotor.setVoltage(4);
    }

    @Override
    public void stop() {
        leftMotor.setVoltage(0);
    }

    @Override
    public void zeroElevatorEncoder() {
        encoder.setPosition(0);
    }

    public double ticksToInches() {
        double ticksPerRev = 42; 
        double shaftRadius = 0.0625;
        double distanceperRev = shaftRadius * Math.PI * 2;
        double currentPosition = encoder.getPosition();
        double conversionFactor = distanceperRev / ticksPerRev;

        return currentPosition * conversionFactor;
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    

}
