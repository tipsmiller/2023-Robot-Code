// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  //TODO tacky fix for autos remove when actual code is here
  public static CommandBase exampleAuto(Drivetrain subsystem, XboxController m_Controller) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new DriveWithGamepad(subsystem, m_Controller));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
