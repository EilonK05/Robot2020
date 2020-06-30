/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.limelight;

public class PIDVisionFeeder extends CommandBase {

  private Chassis chassis;

  public PIDVisionFeeder() {
    chassis = Chassis.getInstance();
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.getInstance().camMode(0);
    limelight.getInstance().pipeline(6);
    chassis.rampRate(0);
    chassis.setidilmodeBrake(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double angel = chassis.anglePIDVisionOutput(0);
    double distacne = chassis.distancePIDVisionOutput(66);
    if(limelight.getInstance().Tshort > 2){
      chassis.ArcadeDrive(angel, distacne);
    }else{
      chassis.tankDrive(0, 0);
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //limelight.getInstance().camMode(1);
    chassis.tankDrive(0, 0);
    chassis.reset();
    chassis.setidilmodeBrake(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return chassis.isPIDVisionOnTargetDistance();
  }
}