/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MAMotorControler;
import frc.robot.utils.MAPidController;
import frc.robot.utils.RobotConstants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private static Shooter shooter;
  private MAMotorControler shooterSparkMaxA;
  private MAMotorControler shooterSparkMaxB;
  private DigitalInput IR;
  private MAPidController pid;

  // PID
  final double SHOOTER_PID_KP = 0.0006;
  final double SHOOTER_PID_KI = 0.0002;
  final double SHOOTER_PID_KD = 0;
  double SHOOTER_PID_KF;

  public Shooter() {
    shooterSparkMaxA = new MAMotorControler(RobotConstants.SPARK_MAX, RobotConstants.m_ID5, RobotConstants.Encoder,
        null, 60, true, 0);
    shooterSparkMaxB = new MAMotorControler(RobotConstants.SPARK_MAX, RobotConstants.m_ID6, 60, false, 0);
    shooterSparkMaxB.followSparkMax(shooterSparkMaxA);

    IR = new DigitalInput(RobotConstants.DIO_ID0);

    pid = new MAPidController(SHOOTER_PID_KP, SHOOTER_PID_KI, SHOOTER_PID_KD, 0, 70, 0, 12);
  }

  // Motor Functions
  public void setShooterMotor(double power) {
    shooterSparkMaxA.setvoltage(power);
  }

  // IR Functions
  public boolean getIR() {
    return IR.get();
  }

  // Encoder Functions
  public double getVelocity() {
    return shooterSparkMaxA.getVelocity();
  }

  // PID Functions
  public double getPID() {
    pid.setF((12 * pid.getSetpoint()) / 5700);
    return pid.calculate(getVelocity());
  }

  public boolean PIDatSetpoint() {
    return pid.atSetpoint(0.1);
  }

  public void resetPID() {
    pid.reset();
  }

  public void setSetpoint(double setpoint) {
    pid.setSetpoint(setpoint);
  }

  // Singletone
  public static Shooter getInstance() {
    if (shooter == null) {
      shooter = new Shooter();
    }
    return shooter;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Encoder", getVelocity());
    SmartDashboard.putBoolean("Shooter IR", getIR());
    SmartDashboard.putNumber("Shooter PID Setpoint", pid.getSetpoint());
    SmartDashboard.putNumber("Shooter PID P", pid.getPositionError());
    SmartDashboard.putNumber("Shooter Motor Output", shooterSparkMaxA.getOutput());
    SmartDashboard.putBoolean("Shooter PID At Setpoint", PIDatSetpoint());
    SmartDashboard.putData("Shooter Subsystem", Shooter.getInstance());
  }
}
