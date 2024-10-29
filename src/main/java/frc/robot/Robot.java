// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.SMF.SubsystemManagerFactory;

public class Robot extends LoggedRobot {
  private RobotContainer m_robotContainer;



  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    Logger.start();

    SubsystemManagerFactory.getInstance().registerSubsystem(m_robotContainer, false);
    SubsystemManagerFactory.getInstance().disableAllSubsystems();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {
    SubsystemManagerFactory.getInstance().disableAllSubsystems();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    SubsystemManagerFactory.getInstance().notifyAutonomousStart();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    SubsystemManagerFactory.getInstance().notifyTeleopStart();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    SubsystemManagerFactory.getInstance().notifyTestStart();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
