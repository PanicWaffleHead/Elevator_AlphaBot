package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.Constants.SetpointConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import static edu.wpi.first.units.Units.*;

public class RobotContainer {

  private final SubsystemFactory subsystemFactory = new SubsystemFactory();
  private final ElevatorSubsystem elevator = subsystemFactory.buildElevator();
  private final ArmSubsystem arm = subsystemFactory.buildArm();

  private final CommandXboxController driverController = new CommandXboxController(
  DriveControlConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(
    DriveControlConstants.OPERATOR_CONTROLLER_PORT);

  public RobotContainer() {
    elevator.setEncoderPositionCommand(0);
    arm.setSpeedCommand(0);
    configureBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
      elevator.setDefaultCommand(elevator.setSpeedCommand(0));
      arm.setDefaultCommand(arm.setSpeedCommand(0));
  }

  private void configureBindings() {
    operatorController.y().onTrue(elevator.goToSetPointCommand(SetpointConstants.L_ONE_HEIGHT.in(Meters)));
    operatorController.b().whileTrue(elevator.setPercentOutCommand(.1));
    operatorController.a().whileTrue(arm.setPercentOutputCommand(.02));
  }

}
