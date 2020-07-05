/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Chassis;

/**
 * Add your docs here.
 */

public class MACommandBinder {

    private static MACommandBinder commandBinder;

    private MACommandBinder() {

    }

    public void runInstantCommand(JoystickButton button, Runnable toRun, Subsystem... subsystems) {
        button.whenPressed(toRun, subsystems);
    }

    public void runConditionalCommandWhenPressed(JoystickButton button, BooleanSupplier condition, CommandBase onTrue,
            CommandBase onFalse) {
        button.whenPressed(new ConditionalCommand(onTrue, onFalse, condition));
    }

    public void runConditionalCommandWhilePressed(JoystickButton button, BooleanSupplier condition, CommandBase onTrue,
    CommandBase onFalse) {
        button.whileHeld(new ConditionalCommand(onTrue, onFalse, condition));
    }

    public void runCommandWhenPressed(JoystickButton button, Runnable toRun, Subsystem... subsystems) {
        button.whenPressed(new RunCommand(toRun, subsystems));
    }

    public void runCommandWhileHeld(JoystickButton button, Runnable toRun, Subsystem... subsystems) {
        button.whileHeld(new RunCommand(toRun, subsystems));
    }

    public void startEndBind(JoystickButton button, Runnable initializeFunction, Runnable endFunction,
            Subsystem... subsystems) {
        button.whileActiveContinuous(new StartEndCommand(initializeFunction, endFunction, subsystems));
    }

    public void startEndBind(JoystickButton button, Runnable initializeFunction, Runnable endFunction,
            BooleanSupplier endCondition, Subsystem... subsystems) {
        button.whileActiveContinuous(
                new StartEndCommand(initializeFunction, endFunction, subsystems).withInterrupt(endCondition));
    }

    public void functionalBind(JoystickButton button, Runnable initializeFunction, Runnable executeFunction,
    Consumer<Boolean> endFunction, BooleanSupplier isFinished, Subsystem... subsystems) {
        button.whileActiveContinuous(
        new FunctionalCommand(initializeFunction, executeFunction, endFunction, isFinished, subsystems));
    }

    public void perpetualBind(JoystickButton button, Command toRun) {
        button.whenPressed(new PerpetualCommand(toRun));
    }

    public void printWhenFinishedBind(JoystickButton button, Command toRun, String message) {
        button.whenPressed(toRun.andThen(new PrintCommand(message)));
    }

    public static MACommandBinder getInstance() {
        if (commandBinder == null) {
            commandBinder = new MACommandBinder();
        }
        return commandBinder;
    }
}
