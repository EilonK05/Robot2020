/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Conveyance.ConveyanceCommand;
import frc.robot.commands.Elevator.ElevatorPiston;
import frc.robot.commands.Intake.IntakeMotor;
import frc.robot.commands.Intake.IntakePiston;
import frc.robot.commands.Shooter.ShooterPID;
import frc.robot.commands.Shooter.ShooterTransfer;

/*
import frc.robot.commands.Roulette.roundTwoRoulettePID;
import frc.robot.commands.Roulette.roundThreeRoulettePID;
import frc.robot.subsystems.Roulette;
*/

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static XboxController OperatingJoystick = new XboxController(2);
  public static Joystick leftJoystick = new Joystick(0);
  public static Joystick rightJoystick = new Joystick(1);

  private JoystickButton aButton = new JoystickButton(OperatingJoystick, 1);
  private JoystickButton bButton = new JoystickButton(OperatingJoystick, 2);
  private JoystickButton xButton = new JoystickButton(OperatingJoystick, 3);
  private JoystickButton yButton = new JoystickButton(OperatingJoystick, 4);
  private JoystickButton lbButton = new JoystickButton(OperatingJoystick, 5);
  private JoystickButton rbButton = new JoystickButton(OperatingJoystick, 6);
  private POVButton POVUp = new POVButton(OperatingJoystick, 0);
  private POVButton POVRight = new POVButton(OperatingJoystick, 90);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    aButton.whileHeld(new ConveyanceCommand(0.5));
    bButton.whileHeld(new ConveyanceCommand(-0.5));
    xButton.whileHeld(new IntakeMotor(0.5));
    yButton.whileHeld(new IntakeMotor(-0.5));
    lbButton.whenPressed(new IntakePiston());
    rbButton.whenPressed(new ElevatorPiston());
    POVUp.whileActiveContinuous(new ShooterPID(1, 1));
    POVRight.whileActiveContinuous(new ShooterTransfer());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
