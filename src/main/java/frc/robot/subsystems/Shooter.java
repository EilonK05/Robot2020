/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.ConstantsShooter;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  public static Shooter shooter; 
  CANSparkMax shooterSparkMaxA;
  CANSparkMax shooterSparkMaxB;
  TalonSRX shooterTalon;
  CANEncoder encoder;
  DigitalInput IR;
  PIDController pid;

  // PID
  final double SHOOTER_PID_KP = 1;
  final double SHOOTER_PID_KI = 1;
  final double SHOOTER_PID_KD = 1;
  final double SHOOTER_PID_KF = 1;
  public Shooter() {
    shooterSparkMaxA = new CANSparkMax(ConstantsShooter.SHOOTER_SPARKMAX_A, MotorType.kBrushless);
    shooterSparkMaxB = new CANSparkMax(ConstantsShooter.SHOOTER_SPARKMAX_B, MotorType.kBrushless);
    shooterSparkMaxB.follow(shooterSparkMaxA);

    shooterTalon = new TalonSRX(ConstantsShooter.SHOOTER_TALON);

    encoder = shooterSparkMaxA.getEncoder();
    encoder.setPositionConversionFactor(1);

    IR = new DigitalInput(ConstantsShooter.SHOOTER_IR);

    pid = new PIDController(SHOOTER_PID_KP, SHOOTER_PID_KI, SHOOTER_PID_KD);
    pid.setTolerance(10);
  }
  // Motor Functions
  public void setShooterMotor(double power){
   shooterSparkMaxA.set(power); 
  }
  public void setTransferMotor(double power){
    shooterTalon.set(ControlMode.PercentOutput, power);
  }
  public double getVoltage(){
    return shooterTalon.getStatorCurrent();
  }
  // IR Functions
  public boolean getIR(){
    return IR.get();
  }
  // Encoder Functions
  public double getVelocity(){
    return encoder.getVelocity();
  }
  // PID Functions
  public double getPID(double setpoint){
    return MathUtil.clamp(pid.calculate(getVelocity(), setpoint) + SHOOTER_PID_KF, -1, 1);
  }
  public boolean PIDatSetpoint(){
    return pid.atSetpoint();
  }
  // Singletone
  public static Shooter getinstance(){
    if (shooter == null){
      shooter = new Shooter();
    }
    return shooter;
    }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Encoder", getVelocity());
    SmartDashboard.putBoolean("Shooter IR", getIR());
  }
}
