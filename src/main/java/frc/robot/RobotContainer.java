

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.PadDrive;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LogitechGamingPad;

import java.util.HashMap;
import java.util.function.Consumer;

import javax.management.InstanceAlreadyExistsException;
import javax.naming.OperationNotSupportedException;
import javax.net.ssl.SSLSocket;


import edu.wpi.first.math.kinematics.SwerveModuleState;

//import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LogitechGamingPad;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.Constants;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem;
  private final LogitechGamingPad pad;
  private final LED led;
  
  private final JoystickButton padA;
  private final JoystickButton padB;
  private final JoystickButton padX;
  private final JoystickButton padY;
  private final JoystickButton rightBumper;
  private final JoystickButton leftBumper;

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    pad = new LogitechGamingPad(0);
    led = new LED();
    // limelight = new Limelight();
   
    
    padA = new JoystickButton(pad, 1);
    padB = new JoystickButton(pad, 2);
    padX = new JoystickButton(pad, 3);
    padY = new JoystickButton(pad, 4);
    rightBumper = new JoystickButton(pad, 6);
    leftBumper = new JoystickButton(pad, 5);

    swerveSubsystem = new SwerveSubsystem();
    swerveSubsystem.setDefaultCommand(new PadDrive(swerveSubsystem, pad, true));
    
    //Configure auto chooser
    SmartDashboard.putData("Auto Chooser", m_chooser);

    configureBindings();
  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    padA.onTrue(new InstantCommand(swerveSubsystem::addRotorPositionsforModules));
    padB.onTrue(new InstantCommand(swerveSubsystem::zeroHeading));
    padY.onTrue(new InstantCommand(swerveSubsystem::configAAcornMode));
    padX.onTrue(new InstantCommand(swerveSubsystem::configSlowMode));
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return swerveSubsystem.followTrajectoryCommand(bluePath, true);

    return m_chooser.getSelected();
  }

  /**
    * Gets the test command
    *
    * @return the command to run in test initial
  //   */
}