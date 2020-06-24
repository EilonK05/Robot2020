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

public class Conveyance extends SubsystemBase {
  /**
   * Creates a new Conveyance.
   */
  private static Conveyance conveyance;

  MAMotorControler conveyanceMotor;
  public Conveyance() {
    conveyanceMotor = new MAMotorControler(RobotConstants.TALON, RobotConstants.m_ID11, 60, false, 0);
  }

  public void setMotor(double power){
    conveyanceMotor.set(power);
  }
  public double getVoltage(){
    return conveyanceMotor.getStatorCurrent();
  }
  public static Conveyance getInstance(){
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
