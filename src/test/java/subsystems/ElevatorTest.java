package subsystems;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorTest {
    static final double DELTA = 1e-2; // acceptable deviation range

    ElevatorSubsystem elevator;

    @BeforeEach
    public void setup() {
        assert HAL.initialize(500, 0);

        elevator = new ElevatorSubsystem();

        /* enable the robot */
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        CommandScheduler.getInstance().enable();

        /* delay ~100ms so the devices can start up and enable */
        Timer.delay(0.100);
    }

    @AfterEach
    public void close() {
        elevator.close();
    }

    @Test
    public void setHeightCommand() {
        double middleHeight = (Elevator.ELEVATOR_MAX_HEIGHT + Elevator.ELEVATOR_MIN_HEIGHT) / 2;
        CommandScheduler.getInstance().schedule(elevator.setHeightCommand(() -> middleHeight));
        CommandScheduler.getInstance().run();

        assert elevator.leaderMotor.getAppliedControl().getName().equals(MotionMagicVoltage.class.getSimpleName());
        assert elevator.getTargetHeight() == middleHeight;
    }
    
    @Test
    public void setMaxHeightCommand() {
        CommandScheduler.getInstance().schedule(elevator.setHeightCommand(() -> Elevator.ELEVATOR_MAX_HEIGHT + 1));
        CommandScheduler.getInstance().run();

        assert elevator.getTargetHeight() == Elevator.ELEVATOR_MAX_HEIGHT;
    }

    @Test
    public void setMinHeightCommand() {
        CommandScheduler.getInstance().schedule(elevator.setHeightCommand(() -> Elevator.ELEVATOR_MIN_HEIGHT - 1));
        CommandScheduler.getInstance().run();

        assert elevator.getTargetHeight() == Elevator.ELEVATOR_MIN_HEIGHT;
    }

    @Test
    public void stowHeightVoltageRequest() {
        elevator.leaderMotor.setPosition(Elevator.ELEVATOR_MIN_HEIGHT + 1);
        
        CommandScheduler.getInstance().schedule(elevator.setHeightCommand(() -> Elevator.ELEVATOR_MIN_HEIGHT - 1));
        CommandScheduler.getInstance().run();

        assert elevator.leaderMotor.getAppliedControl().getName().equals(VoltageOut.class.getSimpleName());
    }

    @Test
    public void hardStopVoltageRequest() {
        elevator.leaderMotor.setPosition(Elevator.ELEVATOR_MAX_HEIGHT - 1);
        
        CommandScheduler.getInstance().schedule(elevator.setHeightCommand(() -> Elevator.ELEVATOR_MAX_HEIGHT));
        CommandScheduler.getInstance().run();
        
        assert elevator.leaderMotor.getAppliedControl().getName().equals(VoltageOut.class.getSimpleName());
    }
}
