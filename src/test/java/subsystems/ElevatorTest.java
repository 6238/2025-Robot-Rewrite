package subsystems;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorTest {
    static final double DELTA = 1e-2; // acceptable deviation range
    static final double SIM_UPDATE_TIME = 0.02; // 20ms simulation step

    ElevatorSubsystem elevator;
    TalonFXSimState leaderMotorSimState;

    @BeforeEach
    public void setup() {
        elevator = new ElevatorSubsystem();

        leaderMotorSimState = elevator.leaderMotor.getSimState();
        
        /* enable the robot */
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        CommandScheduler.getInstance().enable();
    }

    @AfterEach
    public void close() {
        elevator.close();
        CommandScheduler.getInstance().disable();
        CommandScheduler.getInstance().clearComposedCommands();
    }

    /*
     * Tests setting the height of the elevator to the midpoint between the min and max height
     * Uses the elevator.setHeightCommand passing in the midpoint
     * It then checks if the control request is using motion magic voltage and if the target height is equal to the midpoint
     */
    @Test
    public void setHeightCommand() {
        double middleHeight = (Elevator.ELEVATOR_MAX_HEIGHT + Elevator.ELEVATOR_MIN_HEIGHT) / 2;
        CommandScheduler.getInstance().schedule(elevator.setHeightCommand(() -> middleHeight));
        CommandScheduler.getInstance().run();

        assert elevator.leaderMotor.getAppliedControl().getName().equals(MotionMagicVoltage.class.getSimpleName());
        assert elevator.getTargetHeight() == middleHeight;
    }
    
    /*
     * Tests the maximum height of the elevator subsystem
     * Sets the height using the elevator.setHeightCommand and pases in the max height + 1
     * Checks that the target height does not excceed the max elevator height
     */
    @Test
    public void setMaxHeightCommand() {
        CommandScheduler.getInstance().schedule(elevator.setHeightCommand(() -> Elevator.ELEVATOR_MAX_HEIGHT + 1));
        CommandScheduler.getInstance().run();

        assert elevator.getTargetHeight() == Elevator.ELEVATOR_MAX_HEIGHT;
    }

    /*
     * Tests the minimum height of the elevator subsystem
     * Sets the height using the elevator.setHeightCommand and pases in the min height - 1
     * Checks that the target height does not go below the min elevator height
     */
    @Test
    public void setMinHeightCommand() {
        CommandScheduler.getInstance().schedule(elevator.setHeightCommand(() -> Elevator.ELEVATOR_MIN_HEIGHT - 1));
        CommandScheduler.getInstance().run();

        assert elevator.getTargetHeight() == Elevator.ELEVATOR_MIN_HEIGHT;
    }

    /*
     * Checks that when stowing, the elevator applies a constant voltage downward
     * Resets the encoder to get the elevator to be at a height where the elevator will apply voltage
     * Sets the height using the elevator.setHeightCommand going to the minimum height (stow) 
     */
    @Test
    public void stowHeightVoltageRequest() {
        leaderMotorSimState.setRawRotorPosition((Elevator.ELEVATOR_MIN_HEIGHT + 1) * Elevator.ELEVATOR_GEAR_RATIO);

        CommandScheduler.getInstance().schedule(elevator.setHeightCommand(() -> Elevator.ELEVATOR_MIN_HEIGHT));
        CommandScheduler.getInstance().run();

        assert elevator.leaderMotor.getAppliedControl().getName().equals(VoltageOut.class.getSimpleName());
    }

    /*
     * Checks that when near the hardstop, the elevator applies a constant voltage upward
     */
    @Test
    public void hardStopVoltageRequest() {
        leaderMotorSimState.setRawRotorPosition((Elevator.ELEVATOR_MAX_HEIGHT - 1) * Elevator.ELEVATOR_GEAR_RATIO);
        
        elevator.leaderMotor.getPosition().waitForUpdate(SIM_UPDATE_TIME); // wait for motor position to update from simstate

        CommandScheduler.getInstance().schedule(elevator.setHeightCommand(() -> Elevator.ELEVATOR_MAX_HEIGHT));
        CommandScheduler.getInstance().run();

        assert elevator.leaderMotor.getAppliedControl().getName().equals(VoltageOut.class.getSimpleName());
    }

    /*
     * Checks that when near the hardstop, the elevator doesn't apply a constant voltage upward if the target is below its current height
     */
    @Test
    public void hardStopNoVoltageRequestOnLowTarget() {
        leaderMotorSimState.setRawRotorPosition((Elevator.ELEVATOR_MAX_HEIGHT - 1) * Elevator.ELEVATOR_GEAR_RATIO);

        elevator.leaderMotor.getPosition().waitForUpdate(SIM_UPDATE_TIME); // wait for motor position to update from simstate

        CommandScheduler.getInstance().schedule(elevator.setHeightCommand(() -> Elevator.ELEVATOR_MAX_HEIGHT));
        CommandScheduler.getInstance().run();

        assert elevator.leaderMotor.getAppliedControl().getName().equals(VoltageOut.class.getSimpleName());
    }
}
