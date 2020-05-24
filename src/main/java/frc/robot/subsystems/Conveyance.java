/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsConveyance;

public class Conveyance extends SubsystemBase {
  /**
   * Creates a new Conveyance.
   */
  public static Conveyance conveyance;
  
  VictorSPX conveyanceMotor;
  public Conveyance() {
    conveyanceMotor = new VictorSPX(ConstantsConveyance.CONVEYANCE_MOTOR);
  }

  public void setMotor(double power){
    conveyanceMotor.set(ControlMode.PercentOutput, power);
  }

  public static Conveyance getinstance(){
    if (conveyance == null){
      conveyance = new Conveyance();
    }
    return conveyance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
