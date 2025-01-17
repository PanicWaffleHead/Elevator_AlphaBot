package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class ElevatorSubsystem extends SubsystemBase{
    
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputs inputs = new ElevatorIOInputs();

    public static final class ElevatorConstants {}

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    public Command setEncoderPositionCommand(double position) {
        return this.runOnce(() -> elevatorIO.setEncoderPosition(position));
    }

    public Command goToSetPointCommand(double position) {
        return this.runOnce(() -> elevatorIO.setPosition(position));
    }

    public Command setSpeedCommand(double speed) {
        return this.run(() -> elevatorIO.setSpeed(speed));
    }

    public Command setPercentOutCommand(double percentOutput) {
        return this.run(() -> elevatorIO.setPercentOutput(percentOutput));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/position", inputs.position);
        SmartDashboard.putNumber("Elevator/velocity", inputs.velocity);
        SmartDashboard.putNumber("Elevator/appliedVoltage", inputs.appliedVoltage);
        SmartDashboard.putNumber("Elevator/positionSetPoint", inputs.positionSetPoint);
        elevatorIO.updateStates(inputs);
    }

}
