package frc.robot.subsystems.collector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase{

    private CollectorIOInputsAutoLogged inputs;
    private CollectorIO io;

    public Collector(CollectorIO io) {

        this.io = io;
        inputs = new CollectorIOInputsAutoLogged();
    }

    public void updateInputs() {
        io.updateInputs(inputs);
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        io.stop();
    }

    @Override
    public void periodic() {
        updateInputs();
    }
}
