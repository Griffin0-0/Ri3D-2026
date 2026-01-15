// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

// ===== Input Devices ===== //
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.EmergencyStopMechanismsCmd;

// ===== Swerve Specific ===== //
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

// ===== Subsystems ===== //
import frc.robot.subsystems.ShooterSubsystem;

// ===== Commands ===== //


// ===== Constants ===== //
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.QuickAccessConstants;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.util.DriveFeedforwards;



public class RobotContainer {

    // Subsystems
    private final SwerveSubsystem swerveSubsystem;
    private final ShooterSubsystem shooterSubsystem;


    // Control Inputs
    private Joystick operatorController;
    private Joystick driverController;
    private XboxController soloController;

    public RobotContainer() {
        if (QuickAccessConstants.isSoloDrive) {
            soloController = new XboxController(OIConstants.kOperatorControllerPort);
        } else {
            operatorController = new Joystick(OIConstants.kOperatorControllerPort);
            driverController = new Joystick(OIConstants.kDriverControllerPort);
        }

        if (QuickAccessConstants.swerveEnabled) {
            swerveSubsystem = new SwerveSubsystem();
        }

        if (QuickAccessConstants.manipulatorsEnabled) {
            shooterSubsystem = new ShooterSubsystem();
        }

        if (QuickAccessConstants.swerveEnabled) {
            if (QuickAccessConstants.isSoloDrive) {
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                    swerveSubsystem,
                    () -> soloController.getLeftY(),
                    () -> soloController.getLeftX(),
                    () -> soloController.getRightX(),
                    () -> soloController.getLeftBumperButton(),
                    () -> soloController.getRightBumperButton()
                ));
            } else {
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                    swerveSubsystem,
                    () -> driverController.getRawAxis(OIConstants.kDriverYAxis),
                    () -> driverController.getRawAxis(OIConstants.kDriverXAxis),
                    () -> -driverController.getRawAxis(OIConstants.kDriverRotAxis),
                    () -> driverController.getRawButton(1),
                    () -> driverController.getRawButton(1)
                ));
            }
        }


        // Register Named Commands
        // NamedCommands.registerCommand("Reset Gyro", new SequentialCommandGroup(swerveSubsystem.zeroHeading(), swerveSubsystem.zeroEverything()));


        if (QuickAccessConstants.manipulatorsEnabled) {
            configureBindings();
        }

        configureMandatoryBindings();
    }


    public void configureMandatoryBindings() {
        if (QuickAccessConstants.isSoloDrive){
            new Trigger(() -> soloController.getStartButton())
            .onTrue(new SequentialCommandGroup(swerveSubsystem.zeroHeading(), swerveSubsystem.zeroEverything()));
            new Trigger(() -> soloController.getBackButton())
            .whileTrue(new EmergencyStopMechanismsCmd());
        } else {
            new JoystickButton(driverController, OIConstants.kController_start)
            .onTrue(new SequentialCommandGroup(swerveSubsystem.zeroHeading(), swerveSubsystem.zeroEverything()));
            new JoystickButton(operatorController, OIConstants.kController_back)
            .whileTrue(new EmergencyStopMechanismsCmd());           // MUST HAVE INCASE OF EMERGENCY
        }
    }


    private void configureBindings() {
        if (QuickAccessConstants.isSoloDrive) {
            new Trigger(() -> soloController.getYButton())
            .onTrue(new Command() {
                
            })
        }
    }

    // public Command getAutonomousCommand() {
    //     return new PathPlannerAuto("");
    // }
}
