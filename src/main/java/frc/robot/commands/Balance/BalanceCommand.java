/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Balance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Balance;

public class BalanceCommand extends CommandBase {
  /**
   * Creates a new Balance.
   */
  public Balance balance;
   public BalanceCommand() {
    balance = Balance.getinstance();
    addRequirements(balance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.OperatingJoystick.getRawAxis(0) > 0.15 || RobotContainer.OperatingJoystick.getRawAxis(0) < 0.15){
      balance.setMotor(RobotContainer.OperatingJoystick.getRawAxis(0));
    }else{
      balance.setMotor(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    balance.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
