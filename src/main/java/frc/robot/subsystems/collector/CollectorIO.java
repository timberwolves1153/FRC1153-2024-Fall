package frc.robot.subsystems.collector;

import org.littletonrobotics.junction.AutoLog;

public interface CollectorIO {
    @AutoLog
    public class CollectorIOInputs{
        public double appliedVolts = 0;
        public double currentAmps = 0;
    }

    public default void updateInputs(CollectorIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void stop() {}
    
}
