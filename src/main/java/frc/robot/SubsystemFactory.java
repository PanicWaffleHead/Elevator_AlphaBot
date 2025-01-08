package frc.robot;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorHardware;

public class SubsystemFactory {
    

    protected ElevatorSubsystem buildElevator() {
        return new ElevatorSubsystem(new ElevatorHardware());
    }
}
