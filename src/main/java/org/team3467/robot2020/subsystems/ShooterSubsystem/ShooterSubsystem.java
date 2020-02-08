/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team3467.robot2020.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3467.robot2020.Constants.CanConstants;

public class ShooterSubsystem extends SubsystemBase
{
    private CANSparkMax shooterLeftMotor, shooterRightMotor;
    private CANPIDController m_pidController1, m_pidController2;
    private CANEncoder m_encoder1, m_encoder2;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, kVelocity;

    private TalonSRX shooter_intake = new TalonSRX(CanConstants.shooter_intake);

    public ShooterSubsystem()
    {
        shooterLeftMotor = new CANSparkMax(10, MotorType.kBrushless);
        shooterRightMotor = new CANSparkMax(11, MotorType.kBrushless);


        shooterLeftMotor.restoreFactoryDefaults();
        shooterRightMotor.restoreFactoryDefaults();
    
        shooterLeftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        shooterRightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        /*
        * In order to use PID functionality for a controller, a CANPIDController object
        * is constructed by calling the getPIDController() method on an existing
        * CANSparkMax object
        */
        m_pidController1 = shooterLeftMotor.getPIDController();
        m_pidController2 = shooterRightMotor.getPIDController();

        // Encoder object created to display position values
        m_encoder1 = shooterLeftMotor.getEncoder();
        m_encoder2 = shooterRightMotor.getEncoder();

        shooterRightMotor.follow(shooterLeftMotor, true);

        // PID coefficients
        kP = 0.00055; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0.0002; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;

        // set PID coefficients
        m_pidController1.setP(kP);
        m_pidController1.setI(kI);
        m_pidController1.setD(kD);
        m_pidController1.setIZone(kIz);
        m_pidController1.setFF(kFF);
        m_pidController1.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        SmartDashboard.putNumber("Velocity", kVelocity);
        
        shooterLeftMotor.setInverted(true);
        shooter_intake.setInverted(true);
    }

    public void ShooterIntake(double speed, boolean on)
    {
        if (on)
        {
            shooter_intake.set(ControlMode.PercentOutput, speed);
        }
        else
        {
            shooter_intake.set(ControlMode.PercentOutput, 0.0);
        }
    }

    public void SpinShooter(double speed, boolean on)
    {
        // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController1.setP(p); kP = p; }
    if((i != kI)) { m_pidController1.setI(i); kI = i; }
    if((d != kD)) { m_pidController1.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController1.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController1.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController1.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((p != kP)) { m_pidController2.setP(p); kP = p; }
    if((i != kI)) { m_pidController2.setI(i); kI = i; }
    if((d != kD)) { m_pidController2.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController2.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController2.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController2.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
        if (on)
        {
            m_pidController1.setReference(speed, ControlType.kVelocity);

            SmartDashboard.putNumber("Current Velocity 2", m_encoder1.getVelocity());
            SmartDashboard.putNumber("Current Velocity 1", m_encoder2.getVelocity());

            SmartDashboard.putNumber("error", speed - m_encoder2.getVelocity());
        }
        else
        {
            m_pidController1.setReference(0.0, ControlType.kVelocity);

            SmartDashboard.putNumber("Current Velocity 2", m_encoder1.getVelocity());
            SmartDashboard.putNumber("Current Velocity 1", m_encoder2.getVelocity());

            SmartDashboard.putNumber("error", speed - m_encoder2.getVelocity());
        }
    }

}