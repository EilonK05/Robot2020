/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Intake.IntakePiston;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Conveyance;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTransfer;

public class IntakeAutomation extends CommandBase {
  /**
   * Creates a new IntakeAutomation.
   */
  private Automation automation;
  private CommandBase IntakePiston;
  private Intake intake;
  private Conveyance conveyance;
  private ShooterTransfer shooterTransfer;
  public IntakeAutomation() {
    automation = Automation.getinstance();
    addRequirements(automation);

    IntakePiston = new IntakePiston();
    intake = Intake.getinstance();
    conveyance = Conveyance.getinstance();
    shooterTransfer = ShooterTransfer.getinstance();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakePiston.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setMotor(1);
    conveyance.setMotor(1);
    if(Shooter.getinstance().getIR()){
      shooterTransfer.setTransferMotor(0);
      conveyance.setMotor(1);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
