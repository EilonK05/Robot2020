/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MAMotorControler;
import frc.robot.utils.RobotConstants;

public class Balance extends SubsystemBase {
  /**
   * Creates a new Balance.
   */
  public static Balance balance;
  MAMotorControler balanceMotor;
  public Balance() {
    balanceMotor = new MAMotorControler(RobotConstants.SPARK_MAX, RobotConstants.m_ID9, 60, false, 0);
  }

  public void setMotor(double power){
    balanceMotor.set(power);
  }

  public static Balance getinstance(){
    if (balance == null){
      balance = new Balance();
    }
    return balance;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
