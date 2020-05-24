/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  public static Intake intake;
  
  WPI_VictorSPX intakeMotor;
  public DoubleSolenoid intakePiston;
  public Intake() {
    intakeMotor = new WPI_VictorSPX(ConstantsIntake.INTAKE_MOTOR);
    intakePiston = new DoubleSolenoid(ConstantsIntake.INTAKE_PISTON_A, ConstantsIntake.INTAKE_PISTON_B);
  }
  // Piston Functions
  public void setPiston(boolean state) {
    if (state) {
      intakePiston.set(Value.kForward);
    }
    else {
      intakePiston.set(Value.kReverse);
    }
  }
  public boolean getPiston(){
    if(intakePiston.get() == Value.kForward){
      return true;
    } else{
      return false;
    }
  }
  // Motor Functions
  public void setMotor(double power){
    intakeMotor.set(ControlMode.PercentOutput, power);
  }
  // Singletone
  public static Intake getinstance(){
    if (intake == null){
      intake = new Intake();
    }
    return intake;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
