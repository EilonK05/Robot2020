/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsRoulette;

public class Roulette extends SubsystemBase {
  /**
   * Creates a new Roulette.
   */
  public static Roulette roulette;

  TalonSRX rouletteMotor;
  ColorSensorV3 colorSensor;
   public Roulette() {
    rouletteMotor = new TalonSRX(ConstantsRoulette.ROULETTE_MOTOR);
    colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  }
  // Motors Functions
  public void setMotor(double power){
    rouletteMotor.set(ControlMode.PercentOutput, power);
  }
  
  // Singletone
  public static Roulette getinstance(){
    if (roulette == null){
      roulette = new Roulette();
    }
    return roulette;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
