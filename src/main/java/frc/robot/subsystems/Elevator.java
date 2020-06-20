/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MAMotorControler;
import frc.robot.utils.RobotConstants;

public class Elevator extends SubsystemBase {
  /**
   * Creates a new Elevator.
   */
  MAMotorControler elevatorMotor;
  Solenoid elevatorPiston;
  public static Elevator elevator;
  
  public Elevator() {
    elevatorMotor = new MAMotorControler(RobotConstants.SPARK_MAX, RobotConstants.m_ID8, RobotConstants.Alternate_Encoder, null, 60, false, 0);
    elevatorPiston = new Solenoid(RobotConstants.p_ID4);
  }
  // Motors Functions
  public void setMotor(double power){
    elevatorMotor.set(power);
  }
  // Pistons Functions
  public void setPiston(boolean state){
    elevatorPiston.set(state);
  }
  public boolean getPiston(){
    return elevatorPiston.get();
  }
  // Encoder Function
  public double getEncoder(){
    return elevatorMotor.getPosition();
  }
  // Singletone
  public static Elevator getinstance(){
    if (elevator == null){
      elevator = new Elevator();
    }
    return elevator;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder", getEncoder());
    SmartDashboard.putBoolean("Elevator Piston", getPiston());
  }
}
