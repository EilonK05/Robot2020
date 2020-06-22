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

public class ShooterTransfer extends SubsystemBase {
  /**
   * Creates a new ShooterTransfer.
   */
  private static ShooterTransfer shooterTransfer;
  private MAMotorControler transferMotor;
  
  public ShooterTransfer() {
    transferMotor = new MAMotorControler(RobotConstants.TALON, RobotConstants.m_ID7, 60, false, 0);
  }
  // Motors Functions
  public void setTransferMotor(double power){
    transferMotor.set(power);
  }
  public double getVoltage(){
    return transferMotor.getStatorCurrent();
  }
  // Singletone
  public static ShooterTransfer getInstance(){
      if (shooterTransfer == null){
        shooterTransfer = new ShooterTransfer();
      }
      return shooterTransfer;
      }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
