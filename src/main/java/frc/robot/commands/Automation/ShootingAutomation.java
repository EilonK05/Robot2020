/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Shooter.ShooterPID;
import frc.robot.commands.Shooter.ShooterTransferCommand;
import frc.robot.commands.Conveyance.ConveyanceCommand;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTransfer;

public class ShootingAutomation extends CommandBase {
  /**
   * Creates a new ShootingAutomation.
   */
  private Automation automation;
  private CommandBase ShooterTransferCommand, ShooterPID, ConveyanceCommand;
  public ShootingAutomation() {
    automation = Automation.getinstance();
    addRequirements(automation);

    ShooterTransferCommand = new ShooterTransferCommand();
    ShooterPID = new ShooterPID(0);
    ConveyanceCommand = new ConveyanceCommand(0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterPID.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ShooterPID.execute();
    if(Shooter.getinstance().PIDatSetpoint()) {
      ShooterTransferCommand.execute();
      ConveyanceCommand.execute();
    }else{
      ConveyanceCommand.end(true);
      ShooterTransferCommand.end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterPID.end(true);
    ShooterTransferCommand.end(true);
    ConveyanceCommand.end(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
