package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

public class ElevatorSubsystem extends SubsystemBase implements AutoCloseable {
    public TalonFX leaderMotor = new TalonFX(Elevator.LEADER_ELEVATOR_MOTOR_ID, "canivore");
    public TalonFX followerMotor = new TalonFX(Elevator.FOLLOWER_ELEVATOR_MOTOR_ID, "canivore");

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

    private double targetPosition = 0.0;

    public ElevatorSubsystem() {
        TalonFXConfiguration elevatorMotorConfigs = new TalonFXConfiguration();
        elevatorMotorConfigs.Feedback.SensorToMechanismRatio = Elevator.ELEVATOR_GEAR_RATIO;

        Slot0Configs motorConfig = elevatorMotorConfigs.Slot0;
        
        motorConfig.GravityType = GravityTypeValue.Elevator_Static;
        motorConfig.kS = Elevator.kS;
        motorConfig.kG = Elevator.kG;
        motorConfig.kV = Elevator.kV;
        motorConfig.kA = Elevator.kA;
        motorConfig.kP = Elevator.kP;
        motorConfig.kI = Elevator.kI;
        motorConfig.kD = Elevator.kD;

        elevatorMotorConfigs.MotionMagic.MotionMagicCruiseVelocity =
            Elevator.MAX_VELOCITY; // Target cruise velocity of 80 rps
        elevatorMotorConfigs.MotionMagic.MotionMagicAcceleration =
            Elevator.MAX_ACCEL; // Target acceleration of 160 rps/s (0.5 seconds)
        elevatorMotorConfigs.MotionMagic.MotionMagicJerk = Elevator.MAX_JERK;
        leaderMotor.getConfigurator().apply(elevatorMotorConfigs);

        followerMotor.setControl(new Follower(Elevator.LEADER_ELEVATOR_MOTOR_ID, true));

        leaderMotor.setNeutralMode(NeutralModeValue.Brake);
        followerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    private void setHeight(double height) {
        height = Math.max( Math.min( height, Elevator.ELEVATOR_MAX_HEIGHT ), Elevator.ELEVATOR_MIN_HEIGHT );
        leaderMotor.setControl(positionRequest.withPosition(height));
        targetPosition = height;
    }

    public double getTargetHeight() {
        return targetPosition;
    }

    public double getHeight() {
        return leaderMotor.getPosition().getValueAsDouble();
    }

    public Command setHeightCommand(DoubleSupplier height) {
        return runOnce(() -> {
            setHeight(height.getAsDouble());
        });
    }

    @Override
    public void periodic() {
        if (getTargetHeight() < Elevator.ELEVATOR_MIN_HEIGHT+1 && getHeight() < Elevator.ELEVATOR_MIN_HEIGHT + 2) {
            leaderMotor.setVoltage(0);
            return;
        }

        if (getHeight() > Elevator.ELEVATOR_MAX_HEIGHT - 8 && getTargetHeight() > Elevator.ELEVATOR_MAX_HEIGHT - 8) {
            leaderMotor.setVoltage(Elevator.kG_TOP);
            return;
        }

        leaderMotor.setControl(positionRequest.withPosition(targetPosition));
    }

    @Override
    public void close() {
        leaderMotor.close();
        followerMotor.close();
    }
}
