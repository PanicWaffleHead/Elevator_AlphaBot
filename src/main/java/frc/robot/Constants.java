// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.units.measure.Distance;

public final class Constants {

  public static class SetpointConstants {
    public static Distance L_ONE_HEIGHT = Centimeters.of(200); //46 is correct. 
  }

  public static class DriveControlConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DRIVE_DEADBAND = 0.05;
    public static final boolean FIELD_ORIENTED_DRIVE = true;
  }

}
