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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Automation.IntakeAutomation;
import frc.robot.commands.Automation.ShootingAutomation;
import frc.robot.commands.Conveyance.ConveyanceCommand;
import frc.robot.commands.Elevator.ElevatorPiston;
import frc.robot.commands.Intake.IntakeMotor;
import frc.robot.commands.Intake.IntakePiston;
import frc.robot.commands.Roulette.RouletteMotor;
import frc.robot.commands.Roulette.RoulettePiston;
import frc.robot.commands.Shooter.ShooterPID;
import frc.robot.commands.Shooter.ShooterTransferCommand;
import frc.robot.utils.RobotConstants;

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

  private JoystickButton leftJoystick5 = new JoystickButton(leftJoystick, 5);
  private JoystickButton rightJoystick5 = new JoystickButton(rightJoystick, 5);
  private JoystickButton leftJoystick3 = new JoystickButton(leftJoystick, 3);
  private JoystickButton rightJoystick3 = new JoystickButton(rightJoystick, 3);

  private JoystickButton AButton = new JoystickButton(OperatingJoystick, RobotConstants.A);
  private JoystickButton BButton = new JoystickButton(OperatingJoystick, RobotConstants.B);
  private JoystickButton YButton = new JoystickButton(OperatingJoystick, RobotConstants.Y);
  private JoystickButton XButton = new JoystickButton(OperatingJoystick, RobotConstants.X);
  private TriggerL triggerL = new TriggerL();
  private TriggerR triggerR = new TriggerR();
  private JoystickButton LB = new JoystickButton(OperatingJoystick, RobotConstants.LB);
  private JoystickButton RB = new JoystickButton(OperatingJoystick, RobotConstants.RB);
  private JoystickButton backkButton = new JoystickButton(OperatingJoystick, RobotConstants.BACK);
  private JoystickButton startButton = new JoystickButton(OperatingJoystick, RobotConstants.START);
  private JoystickButton stickLeft = new JoystickButton(OperatingJoystick, RobotConstants.STICK_LEFT);
  private JoystickButton stickRight = new JoystickButton(OperatingJoystick, RobotConstants.STICK_RIGHT);

  private POVButton POVUp = new POVButton(OperatingJoystick, RobotConstants.POV_UP);
  private POVButton POVDown = new POVButton(OperatingJoystick, RobotConstants.POV_DOWN);
  private POVButton POVLeft = new POVButton(OperatingJoystick, RobotConstants.POV_LEFT);
  private POVButton POVRight = new POVButton(OperatingJoystick, RobotConstants.POV_RIGHT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    
     AButton.whileHeld(new ConveyanceCommand(1)); 
     BButton.whileHeld(new IntakeMotor(-1));
     XButton.whileHeld(new IntakeMotor(1));
     YButton.whileHeld(new ConveyanceCommand(-1)); 
     RB.whenPressed(new ElevatorPiston());
     LB.whenPressed(new IntakePiston()); 
     //backkButton.whileHeld(command);
     //startButton.whileHeld(command); 
     //stickRight.whileHeld(command);
     //stickLeft.whileActiveContinuous(command);
     triggerL.whileActiveContinuous(new ShooterPID(2000));
     triggerR.whileActiveContinuous(new ShooterTransferCommand());
     
     
     POVUp.whenPressed(new RoulettePiston());
     POVLeft.whileActiveContinuous(new ShootingAutomation());
     POVRight.whileActiveContinuous(new RouletteMotor());
     POVDown.whileActiveContinuous(new IntakeAutomation());
    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (SmartDashboard.getNumber("auto", 1) == 1) {
      return null;
    } else if (SmartDashboard.getNumber("auto", 1) == 2) {
      return null;
    } else if (SmartDashboard.getNumber("auto", 1) == 3) {
      return null;
    } else if (SmartDashboard.getNumber("auto", 1) == 4) {
      return null;
    } else {
      return null;
    }
  }
}

class TriggerL extends Trigger {

  @Override
  public boolean get() {
    return RobotContainer.OperatingJoystick.getRawAxis(RobotConstants.LTriger) > 0.5;
  }

}

class TriggerR extends Trigger {

  @Override
  public boolean get() {
    return RobotContainer.OperatingJoystick.getRawAxis(RobotConstants.RTriger) > 0.5;
  }
}
