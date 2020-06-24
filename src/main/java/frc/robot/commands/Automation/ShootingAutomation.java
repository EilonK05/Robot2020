/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Shooter.ShooterPID;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Conveyance;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTransfer;

public class ShootingAutomation extends CommandBase {
  /**
   * Creates a new ShootingAutomation.
   */
  private Automation automation;
  private CommandBase ShooterPID;
  private ShooterTransfer shooterTransfer;
  private Conveyance conveyance; 
  public ShootingAutomation() {
    automation = Automation.getInstance();
    addRequirements(automation);

    ShooterPID = new ShooterPID(2000);
    shooterTransfer = ShooterTransfer.getInstance();
    conveyance = Conveyance.getInstance();
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
    if(Shooter.getInstance().PIDatSetpoint()) {
      shooterTransfer.setTransferMotor(-1);
      conveyance.setMotor(-1);
    }else{
      conveyance.setMotor(0);
      shooterTransfer.setTransferMotor(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterPID.end(true);
    conveyance.setMotor(0);
    shooterTransfer.setTransferMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
