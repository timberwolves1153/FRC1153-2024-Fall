package frc.robot.subsystems.drive;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Tracer;

public class ModuleIOTalonFX implements ModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder turnAbsoluteEncoder;
    private final Queue<Double> timestampQueue;

    private final StatusSignal<Double> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveCurrent;

    private final StatusSignal<Double> turnAbsolutePosition;
    private final StatusSignal<Double> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<Double> turnVelocity;
    private final StatusSignal<Double> turnAppliedVolts;
    private final StatusSignal<Double> turnCurrent;

    // L2 ratios - change as necessary
    private final double DRIVE_GEAR_RATIO = 5.902777777777778; //(50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private final double TURN_GEAR_RATIO = 150.0 / 7.0;

    private final boolean isTurnMotorInverted = true;
    private final Rotation2d absoluteEncoderOffset;

    public ModuleIOTalonFX(int index) {
        switch (index) {
            case 0:
                driveMotor = new TalonFX(1, "Drive");
                turnMotor = new TalonFX(2, "Drive");
                turnAbsoluteEncoder = new CANcoder(3, "Drive");
                absoluteEncoderOffset = new Rotation2d(-0.112 + Math.PI);
                break;
            case 1:
                    driveMotor = new TalonFX(11, "Drive");
                    turnMotor = new TalonFX(12, "Drive");
                    turnAbsoluteEncoder = new CANcoder(13, "Drive");
                    absoluteEncoderOffset = new Rotation2d(2.565 + Math.PI);
                    break;
            case 2:
                driveMotor = new TalonFX(21, "Drive");
                turnMotor = new TalonFX(22, "Drive");
                turnAbsoluteEncoder = new CANcoder(23, "Drive");
                absoluteEncoderOffset = new Rotation2d(-1.328 + Math.PI);
                break; 
            case 3:
                driveMotor = new TalonFX(31, "Drive");
                turnMotor = new TalonFX(32, "Drive");
                turnAbsoluteEncoder = new CANcoder(33, "Drive");
                absoluteEncoderOffset = new Rotation2d(-1.605 + Math.PI);
                break;
            default:
                throw new RuntimeException("Invalid moduel index");
        }

        var driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotor.getConfigurator().apply(driveConfig);
        setDriveBrakeMode(true);

        var turnConfig = new TalonFXConfiguration();
        turnAbsolutePosition = turnAbsoluteEncoder.getAbsolutePosition();
        turnConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnMotor.getConfigurator().apply(turnConfig);
        setTurnBrakeMode(true);

        turnAbsoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());
        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        drivePosition = driveMotor.getPosition();
        drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveMotor, driveMotor.getPosition());
        driveVelocity = driveMotor.getVelocity();
        driveAppliedVolts = driveMotor.getMotorVoltage();
        driveCurrent = driveMotor.getSupplyCurrent();

        turnPosition = turnMotor.getPosition();
        turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnMotor, turnMotor.getPosition());
        turnVelocity = turnMotor.getVelocity();
        turnAppliedVolts = turnMotor.getMotorVoltage();
        turnCurrent = turnMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            Module.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            turnAbsolutePosition,
            turnVelocity,
            turnAppliedVolts,
            turnCurrent);
        driveMotor.optimizeBusUtilization();
        turnMotor.optimizeBusUtilization();
      }


        public void updateInputs(ModuleIOInputs inputs) {
            BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent,
                turnAbsolutePosition,
                turnPosition,
                turnVelocity,
                turnAppliedVolts,
                turnCurrent);
        
            inputs.drivePositionRad =
                Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
            inputs.driveVelocityRadPerSec =
                Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
            inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
            inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};
            inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
            inputs.absoluteEncoderRadians = turnAbsolutePosition.getValueAsDouble() * 2 * Math.PI;
            inputs.turnPosition =
                Rotation2d.fromRotations(turnPosition.getValueAsDouble() / TURN_GEAR_RATIO);
            inputs.turnVelocityRadPerSec =
                Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / TURN_GEAR_RATIO;
            inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
            inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};
        
            inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
            inputs.odometryDrivePositionsRad =
                drivePositionQueue.stream()
                    .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
                    .toArray();
            inputs.odometryTurnPositions =
                turnPositionQueue.stream()
                    .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
                    .toArray(Rotation2d[]::new);
            timestampQueue.clear();
            drivePositionQueue.clear();
            turnPositionQueue.clear();
          }
        @Override
        public void setDriveVoltage(double volts) {
            driveMotor.setControl(new VoltageOut(volts));
        }

    @Override
    public void setTurnVoltage(double volts) {
        turnMotor.setControl(new VoltageOut(volts));
  }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        var config = new MotorOutputConfigs();
        config.Inverted = InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(config);
  }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        var config = new MotorOutputConfigs();
        config.Inverted =
            isTurnMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        turnMotor.getConfigurator().apply(config);
  }
        
        
    }

