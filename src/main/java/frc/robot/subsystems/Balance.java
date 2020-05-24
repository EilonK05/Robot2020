/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstatnsBalance;

public class Balance extends SubsystemBase {
  /**
   * Creates a new Balance.
   */
  public static Balance balance;
  CANSparkMax balanceMotor;
  public Balance() {
    balanceMotor = new CANSparkMax(ConstatnsBalance.BALANCE_MOTOR, MotorType.kBrushless);
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
