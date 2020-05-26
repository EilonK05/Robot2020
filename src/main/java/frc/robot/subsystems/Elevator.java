/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsElevator;

public class Elevator extends SubsystemBase {
  /**
   * Creates a new Elevator.
   */
  CANSparkMax elevatorMotor;
  CANEncoder encoder;
  Solenoid elevatorPiston;
  public static Elevator elevator;
  
  public Elevator() {
    elevatorMotor = new CANSparkMax(ConstantsElevator.ELEVATOR_MOTOR, MotorType.kBrushless);
    elevatorPiston = new Solenoid(ConstantsElevator.ELEVATOR_PISTON);
    encoder = elevatorMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 1);
    encoder.setPositionConversionFactor(1);
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
    return encoder.getPosition();
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
