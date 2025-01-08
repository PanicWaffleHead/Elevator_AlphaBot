package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    
    private final ElevatorIO elevatorIO;

    public static final class ElevatorConstants {
        public static double L_ONE_HEIGHT = 0.43;
        public static final int LEFT_ELEVATOR_MOTOR_ID = 7;
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 10; 
    }

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }
    
    public Command goToSetPointCommand(double position) {
        return this.runOnce(() -> goToSetPoint(position));
    }

    public void goToSetPoint(double position) {
        return;
    }

    public Command setSpeed(double speed) {
        return this.runOnce(() -> elevatorIO.setSpeed(speed));
    }

}
