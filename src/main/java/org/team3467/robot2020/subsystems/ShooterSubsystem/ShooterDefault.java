/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team3467.robot2020.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.team3467.robot2020.subsystems.ShooterSubsystem.ShooterSubsystem;
import org.team3467.robot2020.control.XboxController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShooterDefault extends CommandBase
{
    private final ShooterSubsystem m_shooter;
    private final XboxController m_controller;

    public ShooterDefault(final ShooterSubsystem subsystem, final XboxController controller)
    {
        m_shooter = subsystem;
        m_controller = controller;
        addRequirements(m_shooter);
    }

    @Override
    public void execute()
    {
        m_shooter.ShooterIntake(-1.0 * SmartDashboard.getNumber("Shooter Intake Velocity", 0), m_controller.getYButton());
        m_shooter.SpinShooter(SmartDashboard.getNumber("Shooter Velocity", 0), m_controller.getAButton());
    }
}
