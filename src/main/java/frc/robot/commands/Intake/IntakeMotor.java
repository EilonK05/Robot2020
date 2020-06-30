/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Intake;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeMotor extends CommandBase {
  /**
   * Creates a new IntakeMotor.
   */
  private double power;
  private NetworkTableEntry intakeSpeed;
  private Intake intake;
  public IntakeMotor() {
    intake = Intake.getInstance();
    addRequirements(intake);
    //this.power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSpeed = Shuffleboard.getTab("Systems Control")
  .add("Intake Speed", 1)
  .withWidget("Number Slider")
  .withPosition(1, 1)
  .withSize(2, 1)
  .getEntry();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setMotor(intakeSpeed.getDouble(0));
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
