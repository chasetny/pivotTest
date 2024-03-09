// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveCommands.FieldCentricFacingAngleFix;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.Constants.LimeLightConstants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTag;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;



public class RobotContainer {

  //Controllers
   /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController xbox = new CommandXboxController(0); // Xbox Controller

  //pivot Subsystem
  public final PivotSubsystem pivot = new PivotSubsystem();


  private void configureBindings() {
 
      //pivot
      pivot.setDefaultCommand(pivot.stop());
      // xbox.a().whileTrue(pivot.low());
      // xbox.b().whileTrue(pivot.mid());
      xbox.pov(0).whileTrue(pivot.low());
      xbox.pov(180).whileTrue(pivot.lowSpeedDown());

      xbox.a().whileTrue(pivot.AutoAim(
        LimeLightConstants.goalHeightInches, 
        LimeLightConstants.heightOfShooter,
        LimeLightConstants.limelightLensHeightInches,
        pivot.getDistance(), 
        LimeLightConstants.distanceFromShooter));
      
     
  }

  public RobotContainer() {
  configureBindings();

}

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return null;
    //return runAuto;
  }
}
