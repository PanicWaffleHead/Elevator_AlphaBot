package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorConstants;
import static edu.wpi.first.units.Units.*;

public class RobotContainer {

  private final SubsystemFactory subsystemFactory = new SubsystemFactory();
  private final ElevatorSubsystem elevator = subsystemFactory.buildElevator();

  private final CommandXboxController driverController = new CommandXboxController(
  DriveControlConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(
    DriveControlConstants.OPERATOR_CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
      elevator.setDefaultCommand(elevator.setSpeed(0));
  }

  private void configureBindings() {
    operatorController.y().onTrue(elevator.goToSetPointCommand(ElevatorConstants.L_ONE_HEIGHT.in(Meters)));
  }

}
