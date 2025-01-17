package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO;

public class ArmSubsystem extends SubsystemBase {
    private final ArmIO armIO;

    public ArmSubsystem(ArmIO armIO) {
        this.armIO = armIO;
    }

    public Command setSpeedCommand(double speed) {
        return this.run(() -> armIO.setSpeed(speed));
    }    
    
}
