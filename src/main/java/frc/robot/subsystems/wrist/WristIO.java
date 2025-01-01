package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface WristIO {
    @AutoLog
    public class WristInputs {
        public double positionDegrees = 0;
        public Rotation2d absolutePosition = new Rotation2d();
        public double absolutePositionRadians = 0;
        public double appliedVolts = 0;
    }

    public default void updateInputs(WristInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void stop() {}

    public default void holdPosition() {}

    public default void setTargetPosition(double degrees) {}
}
