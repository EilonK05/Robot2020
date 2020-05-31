/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Conveyance.ConveyanceCommand;
import frc.robot.commands.Intake.IntakeMotor;
import frc.robot.commands.Intake.IntakePiston;
import frc.robot.commands.Shooter.ShooterTransferCommand;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class IntakeTestAutomation extends ParallelDeadlineGroup {
  /**
   * Creates a new IntakeTestAutomation.
   */
  public IntakeTestAutomation() {
    // Add your commands in the super() call.  Add the deadline first.
    super(new WaitUntilCommand(Shooter.getinstance()::getIR),
    new SequentialCommandGroup(
      new IntakePiston(), 
      new IntakeMotor(0.5)), 
    new ConveyanceCommand(0.5), 
    new ShooterTransferCommand()
  );
  }
}
