/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team3467.robot2020.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team3467.robot2020.subsystems.DriveSubsystem.DriveSubsystem;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class TankDrive extends CommandBase {
  
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward The control input for driving forwards/backwards
   * @param rotation The control input for turning
   */
  public TankDrive(final DriveSubsystem subsystem, final DoubleSupplier left, final DoubleSupplier right) {
    m_drive = subsystem;
    m_left = left;
    m_right = right;
    addRequirements(m_drive);
  }


  @Override
  public void execute() {
    m_drive.tankDrive((-1)*m_left.getAsDouble(), (-1)*m_right.getAsDouble());
  }

}
