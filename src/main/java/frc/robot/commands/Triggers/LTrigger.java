/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Triggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

/**
 * Add your docs here.
 */
public class LTrigger extends edu.wpi.first.wpilibj2.command.button.Trigger {
  @Override
  public boolean get() {
    return RobotContainer.OperatingJoystick.getRawAxis(2) > 0.5;
  }
}
