/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Intake.IntakeMotor;
import frc.robot.utils.MAMotorControler;
import frc.robot.utils.RobotConstants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  public static Intake intake;
  
  private MAMotorControler intakeMotor;
  private DoubleSolenoid intakePiston;
  
  public Intake() {
    intakeMotor = new MAMotorControler(RobotConstants.VICTOR, RobotConstants.m_ID12, 60, false, 0);
    intakePiston = new DoubleSolenoid(RobotConstants.p_ID7, RobotConstants.p_ID5);
  }
  // Piston Functions
  public void setForward(){
    intakePiston.set(Value.kForward);
  }
  public void setReverse(){
    intakePiston.set(Value.kReverse);
  }

  public Value getPiston() {
    return intakePiston.get();
  }

  // Motor Functions
  public void setMotor(double power){
    intakeMotor.set(power);
  }
  // Singletone
  public static Intake getInstance(){
    if (intake == null){
      intake = new Intake();
    }
    return intake;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData("Intake Forward", new IntakeMotor(1));
    SmartDashboard.putData("Intake Backward", new IntakeMotor(-1));
  }
}
