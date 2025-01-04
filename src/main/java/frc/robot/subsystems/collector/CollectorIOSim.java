package frc.robot.subsystems.collector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class CollectorIOSim implements CollectorIO{

    private FlywheelSim sim;
    private DCMotor neo;

    public CollectorIOSim() {

        sim = new FlywheelSim(DCMotor.getNeo550(1), 1, 1);

    }

    @Override
    public void updateInputs(CollectorIOInputs inputs) {
        inputs.appliedVolts = DCMotor.getNeo550(1).nominalVoltageVolts;
        inputs.currentAmps = DCMotor.getNEO(1).getCurrent(10, inputs.appliedVolts);
    }

    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
    }

    @Override
    public void stop() {
        sim.setInputVoltage(0);
    }


    
}
