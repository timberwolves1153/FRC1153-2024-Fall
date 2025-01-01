package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristIOSim implements WristIO {

    private SingleJointedArmSim wristSim;

    
    
    public WristIOSim() {

        wristSim = new SingleJointedArmSim(DCMotor.getNEO(1), 44, 650, Units.inchesToMeters(22), 0, Math.PI/2, false, 0);
    }
}
