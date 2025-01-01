package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase{

    private WristIO io;
    private WristInputsAutoLogged inputs;

    private ProfiledPIDController profiledPID;
    private TrapezoidProfile.Constraints wristConstraints;
    private ArmFeedforward ff;

    private Mechanism2d mech2d;
    

    public Wrist(WristIO io) {
        this.io = io;
        inputs = new WristInputsAutoLogged();
        wristConstraints = new Constraints(2, 2);

        profiledPID = new ProfiledPIDController(0.01, 0, 0, wristConstraints);
        ff = new ArmFeedforward(0.01, 0, 0);

    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void stop() {
        io.stop();
    }

    public void setTargetPosition(double degrees) {
        profiledPID.setGoal(Units.degreesToRadians(degrees));
        
        io.setVoltage(
            profiledPID.calculate(
                inputs.absolutePositionRadians, 
                Units.degreesToRadians(degrees)) 
            + ff.calculate(
                Units.degreesToRadians(degrees), 
                profiledPID.getSetpoint().velocity));
    }

    public void holdPosition() {
        profiledPID.setGoal(inputs.absolutePositionRadians);

        io.setVoltage(
            profiledPID.calculate(
                inputs.absolutePositionRadians, 
                inputs.absolutePositionRadians) 
            + ff.calculate(
                inputs.absolutePositionRadians, 
                profiledPID.getSetpoint().velocity));

    }
    
}
