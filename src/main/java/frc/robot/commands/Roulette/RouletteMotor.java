/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Roulette;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConstantsRoulette;
import frc.robot.subsystems.Roulette;

public class RouletteMotor extends CommandBase {
  /**
   * Creates a new RouletteMotor.
   */
  Roulette roulette;
  Color wantedColor;
  int setpoint;
  double lastTimeOnTarget;
  double waitTime;
  
  public RouletteMotor() {
    roulette = Roulette.getInstance();

    addRequirements(roulette);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    String gameData = DriverStation.getInstance().getGameSpecificMessage();

    switch (gameData.charAt(0)) {
      case 'r' :
      wantedColor = ConstantsRoulette.Red;
      break;
      case 'b' :
        wantedColor = ConstantsRoulette.Blue;
        break;
      case 'y':
        wantedColor = ConstantsRoulette.Yellow;
        break;
      case 'g':
        wantedColor = ConstantsRoulette.Green;
        break;  
    }

    int optimalWay = roulette.getOptimalWay(wantedColor);

    roulette.setSetpoint(roulette.getColorEncoder() + optimalWay);
    roulette.setReversed(optimalWay < 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = roulette.getPID();

    roulette.setMotor(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    roulette.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
