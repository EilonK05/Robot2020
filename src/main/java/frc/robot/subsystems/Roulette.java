/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.ConstantsRoulette;
import frc.robot.utils.MAMotorControler;
import frc.robot.utils.RobotConstants;

public class Roulette extends SubsystemBase {
  /**
   * Creates a new Roulette.
   */
  private static Roulette roulette;
  private MAMotorControler rouletteVictor;
  private Solenoid roulettePiston;

  private PIDController colorPID;
  private ColorSensorV3 colorSensor;
  private ColorMatch colorMatcher;

  private Color rouletteColors[] = {ConstantsRoulette.Red, ConstantsRoulette.Yellow, ConstantsRoulette.Blue, ConstantsRoulette.Green};
  private ColorMatchResult closestColor;
  private Color lastColor;
  
  private boolean isReversed;
  private int colorEncoder;


   public Roulette() {
    rouletteVictor = new MAMotorControler(RobotConstants.VICTOR, RobotConstants.m_ID10, 60, false, 0);
    colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    roulettePiston = new Solenoid(RobotConstants.p_ID6);

    colorMatcher = new ColorMatch();

    for (Color color : rouletteColors) {
      colorMatcher.addColorMatch(color);
    }
  }
  // Motors Functions
  public void setMotor(double power){
    rouletteVictor.set(power);
  }
  // Piston Functions
  public void setPiston(boolean state){
    roulettePiston.set(state);
  }
  public boolean getPiston(){
    return roulettePiston.get();
  }
  // Color Functions
  public Color getColor(){
    return colorSensor.getColor();
  }

  public int LinearSearch(Color[] Colors, Color wantedColor) {
    for (int i=0; i < Colors.length; i++) {
      if (Colors[i] == wantedColor) {
        return i;
      } 
    }
    return 0;
  }

  public int getOptimalWay(Color wantedColor) {
    
    int currentColorIndex = LinearSearch(rouletteColors, closestColor.color);
    int wantedColorIndex = LinearSearch(rouletteColors, wantedColor);
  
    int positive_way = currentColorIndex + wantedColorIndex;
    int negative_way = currentColorIndex - (rouletteColors.length - wantedColorIndex);

    return Math.abs(positive_way) > Math.abs(negative_way) ? negative_way : positive_way;
  }

  public void  setSetpoint(int setpoint) {
    colorPID.setSetpoint(setpoint);
  }

  public double getPID() {
    return MathUtil.clamp(colorPID.calculate(colorEncoder), -1, 1);
  }

  public boolean atSetpoint() {
    return colorPID.atSetpoint();
  }

  public int getColorEncoder() {
    return colorEncoder;
  }

  public void setReversed(boolean state) {
    isReversed = state;
  }

  // Singletone
  public static Roulette getInstance(){
    if (roulette == null){
      roulette = new Roulette();
    }
    return roulette;
  }

  public void printDashboard() {
    SmartDashboard.putBoolean("Roulette solenoid state:", roulettePiston.get());

    SmartDashboard.putNumber("Sensor Red:", getColor().red);
    SmartDashboard.putNumber("Sensor Green:", getColor().green);
    SmartDashboard.putNumber("Sensor Blue:", getColor().blue);
    
    SmartDashboard.putNumber("Roulette Closest Red:", closestColor.color.red);
    SmartDashboard.putNumber("Roulette Closest Green:", closestColor.color.green);
    SmartDashboard.putNumber("Roulette Closest Blue:", closestColor.color.blue);

    //SmartDashboard.putNumber("Roulette PID Setpoint", colorPID.getSetpoint());
  }
  
  @Override
  public void periodic() {
    closestColor = colorMatcher.matchClosestColor(getColor());

    if (closestColor.color != lastColor) {
      if (isReversed) {
        colorEncoder--;
      }
      else {
        colorEncoder++;
      }
    }
    lastColor = closestColor.color;

    printDashboard();
  }
}
