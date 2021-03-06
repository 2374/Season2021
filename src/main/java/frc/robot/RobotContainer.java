// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.commands.AutonomousDriveCommand;
import frc.robot.commands.ComplexDriveAuto;
// import frc.robot.commands.DriveCommand;
import frc.robot.commands.InteractiveDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final InteractiveDriveCommand m_idriveCommand = new InteractiveDriveCommand(m_drivetrainSubsystem);
  // private final AutonomousDriveCommand m_adriveCommand = new AutonomousDriveCommand(m_drivetrainSubsystem);
  private final ComplexDriveAuto m_adriveCommand = new ComplexDriveAuto(m_drivetrainSubsystem);
  private final Command m_driveComand = m_adriveCommand.ComplexDriveAuton();
  private XboxController xbox = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drivetrainSubsystem.setDefaultCommand(m_idriveCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    System.out.println("ROSS init buttons");
    new JoystickButton(xbox, 7).whenPressed(new InstantCommand(() -> m_drivetrainSubsystem.resetGyroscope()));
    new JoystickButton(xbox, 5).whenPressed(new InstantCommand(() -> m_drivetrainSubsystem.left90Turn()));
    new JoystickButton(xbox, 6).whenPressed(new InstantCommand(() -> m_drivetrainSubsystem.right90Turn()));
  }

  public void executeAutoCommands() {
    m_adriveCommand.execute();
    // m_drivetrainSubsystem.drive(new Translation2d(0.2, 0.0), 0.0, true);
    // m_drivetrainSubsystem.drive(new Translation2d(0.3, 0.0), 0.0, true);
    // m_drivetrainSubsystem.drive(new Translation2d(0.4, 0.0), 0.0, true);
    // m_drivetrainSubsystem.drive(new Translation2d(0.3, 0.0), 0.0, true);
    // Timer.delay(2.0);
    // m_drivetrainSubsystem.drive(new Translation2d(0.0, 0.0), 0.0, true);

  }

  /**
  * Use this to pass the autonomous command to the main {@link Robot} class.
  *
  * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
  // An ExampleCommand will run in autonomous
  System.out.println("ROSS get autonomous command");
  return m_driveComand;
  }

  // public void executeAutoCommands() {
  // // m_drivetrainSubsystem.autonomousPeriodic();
  // }
}
