package frc.robot.subsystems.elevator;

import java.util.logging.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    
    public ElevatorInputsAutoLogged inputs;
    public ElevatorIO io;

    

    public Elevator(ElevatorIO io) {

        this.io = io;
        inputs = new ElevatorInputsAutoLogged();
        
        Mechanism2d mech2d = new Mechanism2d(Units.inchesToMeters(35), Units.inchesToMeters(60));
  MechanismRoot2d root2d = mech2d.getRoot("Elevator Root", 0, Units.inchesToMeters(0));
  MechanismLigament2d elevatorLig2d =
      root2d.append(
          new MechanismLigament2d(
              "Elevator",
              20,
              0,
              15,
              new Color8Bit(Color.kPurple)));
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        io.stop();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        
    }
}
