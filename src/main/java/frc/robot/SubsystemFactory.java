package frc.robot;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.arm.ArmHardware;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorHardware;

public class SubsystemFactory {
    

    protected ElevatorSubsystem buildElevator() {
        return new ElevatorSubsystem(new ElevatorHardware());
    }

    protected ArmSubsystem buildArm() {
        return new ArmSubsystem(new ArmHardware());
    }
}
