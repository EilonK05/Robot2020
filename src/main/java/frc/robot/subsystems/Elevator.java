/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsElevator;

public class Elevator extends SubsystemBase {
  /**
   * Creates a new Elevator.
   */
  CANSparkMax elevatorMotor;
  CANEncoder encoder;
  DoubleSolenoid elevatorPiston;
  public static Elevator elevator;
  
  public Elevator() {
    elevatorMotor = new CANSparkMax(ConstantsElevator.ELEVATOR_MOTOR, MotorType.kBrushless);
    elevatorPiston = new DoubleSolenoid(ConstantsElevator.ELEVATOR_PISTON_A, ConstantsElevator.ELEVATOR_PISTON_B);
    encoder = elevatorMotor.getEncoder();
    encoder.setPositionConversionFactor(1);
  }
  // Motors Functions
  public void setMotor(double power){
    elevatorMotor.set(power);
  }
  // Pistons Functions
  public void setPiston(boolean state) {
    if (state) {
      elevatorPiston.set(Value.kForward);
    }
    else {
      elevatorPiston.set(Value.kReverse);
    }
  }
  public boolean getPiston(){
    if(elevatorPiston.get() == Value.kForward){
      return true;
    } else{
      return false;
    }
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
    // This method will be called once per scheduler run
  }
}
