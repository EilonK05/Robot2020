/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsShooter;

public class ShooterTransfer extends SubsystemBase {
  /**
   * Creates a new ShooterTransfer.
   */
  public static ShooterTransfer shooterTransfer;
  TalonSRX transferMotor;
  public ShooterTransfer() {
    transferMotor = new TalonSRX(ConstantsShooter.SHOOTER_TALON);
  }
  // Motors Functions
  public void setTransferMotor(double power){
    transferMotor.set(ControlMode.PercentOutput, power);
  }
  public double getVoltage(){
    return transferMotor.getStatorCurrent();
  }
  // Singletone
  public static ShooterTransfer getinstance(){
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
