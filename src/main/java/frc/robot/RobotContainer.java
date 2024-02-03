// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final SendableChooser<Command> autoChooser;


  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  //April Tag T^T
  private Rotation2d m_rotation = new Rotation2d();

  SwerveRequest.FieldCentricFacingAngle FieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle();


  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);

    joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
  
    // Limelight data T^T
    var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;


    //Rotate to face April Tag T^T
    joystick.rightTrigger().whileTrue(
     
      drivetrain.applyRequest(() -> FieldCentricFacingAngle.withVelocityX(-joystick.getLeftY() * MaxSpeed)
      .withVelocityY(-joystick.getLeftX() * MaxSpeed)
      .withTargetDirection(new Rotation2d(0))) //Force Set
      //.withTargetDirection(lastResult.getBotPose2d().getRotation())) //April Tag T^T
      );
      
    //Align with April Tag T^T
    joystick.leftTrigger().whileTrue(  
     
      drivetrain.applyRequest(() -> FieldCentricFacingAngle.withVelocityX(lastResult.getBotPose2d().getX())
      .withVelocityY(lastResult.getBotPose2d().getY())
      .withTargetDirection(lastResult.getBotPose2d().getRotation())));
      


  }

  public RobotContainer() {
    configureBindings();
       autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
    //return runAuto;
  }
}
