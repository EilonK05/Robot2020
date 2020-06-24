/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Balance.BalanceCommand;
import frc.robot.commands.Chassis.MAPath;
import frc.robot.commands.Chassis.tankDrive;
import frc.robot.commands.Elevator.ElevatorMotor;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Balance;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Conveyance;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Roulette;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.limelight;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  // private frc.robot.commands.Chassis.tankDrive tankDrive = new
  // frc.robot.commands.Chassis.tankDrive();
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    limelight.getInstance();
    Chassis.getInstance();
    Balance.getInstance();
    Shooter.getInstance();
    Elevator.getInstance();
    Conveyance.getInstance();
    Roulette.getInstance();
    Automation.getInstance();
    Autonomous.getInstance();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
    CommandScheduler.getInstance().setDefaultCommand(Elevator.getInstance(), new ElevatorMotor());
    CommandScheduler.getInstance().setDefaultCommand(Balance.getInstance(), new BalanceCommand());
    CommandScheduler.getInstance().setDefaultCommand(Chassis.getInstance(), new tankDrive());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    // Chassis.getInstance().setidilmodeBrake(true);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // MAPath.pathnum = 0;
    Chassis.getInstance().resetValue();
    m_autonomousCommand = new MAPath(0.1);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    Chassis.getInstance().setidilmodeBrake(false);
    Chassis.getInstance().rampRate(0);
    //CommandScheduler.getInstance().setDefaultCommand(Chassis.getInstance(), ankDrive);
    Chassis.getInstance().resetValue();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();

    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
