/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Conveyance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyance;

public class ConveyanceCommand extends CommandBase {
  /**
   * Creates a new ConveyanceCommand.
   */
  private double power;
  public Conveyance conveyance;
  public ConveyanceCommand(double power) {
    conveyance = Conveyance.getInstance();
    addRequirements(conveyance);
    this.power = power;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyance.setMotor(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyance.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
