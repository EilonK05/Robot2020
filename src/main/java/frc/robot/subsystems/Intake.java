package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MAMotorControler;
import frc.robot.utils.RobotConstants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  public static Intake intake;
  
  private MAMotorControler intakeMotor;
  private DoubleSolenoid intakePiston;
  
  public Intake() {
    intakeMotor = new MAMotorControler(RobotConstants.VICTOR, RobotConstants.m_ID12, 60, false, 0);
    intakePiston = new DoubleSolenoid(RobotConstants.p_ID7, RobotConstants.p_ID5);
  }

  public void setPiston(boolean state) {
    if (state) {
      intakePiston.set(Value.kForward);
    }
    else {
    intakePiston.set(Value.kReverse);
    }
  }
  
  public void reversePiston() {
    setPiston(!getPiston());
  }
  // Piston Functions
  public void setForward(){
    intakePiston.set(Value.kForward);
  }
  public void setReverse(){
    intakePiston.set(Value.kReverse);
  }

  public boolean getPiston() {
    return intakePiston.get() == Value.kForward;
  }

  // Motor Functions
  public void setMotor(double power){
    intakeMotor.set(power);
  }

  public Runnable getMotorFunction(double power) {
    return () -> setMotor(power);
  }
  // Singletone
  public static Intake getInstance(){
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