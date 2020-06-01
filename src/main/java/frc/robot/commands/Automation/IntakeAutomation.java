/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Conveyance.ConveyanceCommand;
import frc.robot.commands.Intake.IntakeMotor;
import frc.robot.commands.Intake.IntakePiston;
import frc.robot.commands.Shooter.ShooterTransferCommand;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Shooter;

public class IntakeAutomation extends CommandBase {
  /**
   * Creates a new IntakeAutomation.
   */
  private Automation automation;
  private CommandBase IntakePiston, IntakeMotor, ConveyanceCommand, ShooterTransfer;
  public IntakeAutomation() {
    automation = Automation.getinstance();
    addRequirements(automation);

    IntakePiston = new IntakePiston();
    IntakeMotor = new IntakeMotor(0.5);
    ConveyanceCommand = new ConveyanceCommand(0.5);
    ShooterTransfer = new ShooterTransferCommand();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakePiston.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeMotor.execute();
    ConveyanceCommand.execute();
    if(Shooter.getinstance().getIR()){
      ShooterTransfer.end(true);
      ConveyanceCommand.end(false);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeMotor.end(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
