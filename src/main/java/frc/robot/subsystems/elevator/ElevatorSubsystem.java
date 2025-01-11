package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class ElevatorSubsystem extends SubsystemBase{
    
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputs inputs = new ElevatorIOInputs();

    public static final class ElevatorConstants {
        public static Distance L_ONE_HEIGHT = Centimeters.of(46); 
        public static final int LEFT_ELEVATOR_MOTOR_ID = 7;
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 10; 
        public static final double METERS_PER_REVOLUTION = Units.inchesToMeters(27) / 41.951946;
        public static final Distance ALLOWED_SETPOINT_ERROR = Inches.of(1); 
        public static final LinearVelocity MAX_VEL = MetersPerSecond.of(0.8);
        public static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(0.4);
    }

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }
    
    public Command goToSetPointCommand(double position) {
        return this.runOnce(() -> elevatorIO.setPosition(position));
    }

    public Command setSpeed(double speed) {
        return this.run(() -> elevatorIO.setSpeed(speed));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/position", inputs.position);
        SmartDashboard.putNumber("Elevator/velocity", inputs.velocity);
        SmartDashboard.putNumber("Elevator/appliedVoltage", inputs.appliedVoltage);
        SmartDashboard.putNumber("Elevator/positionSetPoint", inputs.positionSetPoint);
        elevatorIO.updateInputs(inputs);
    }

}
