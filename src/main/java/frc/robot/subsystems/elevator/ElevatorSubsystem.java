package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    
    private final ElevatorIO elevatorIO;

    public static final class ElevatorConstants {
        public static double L_ONE_HEIGHT = 0.43;
        public static final int LEFT_ELEVATOR_MOTOR_ID = 7;
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 10; 
        public static final double METERS_PER_REVOLUTION = Units.inchesToMeters(27) / 41.951946;
        public static final double ALLOWED_SETPOINT_ERROR = Units.inchesToMeters(1);
        public static final double MAX_VEL = 0.8;
        public static final double MAX_ACCEL = 0.4;
    }

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }
    
    public Command goToSetPointCommand(double position) {
        return this.runOnce(() -> elevatorIO.setPosition(position));
    }

    public Command setSpeed(double speed) {
        return this.runOnce(() -> elevatorIO.setSpeed(speed));
    }

}
