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
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final SendableChooser<Command> autoChooser;

  PIDController turnPID = new PIDController(0.1,0.01 ,0 );
  PIDController drivePID = new PIDController(0.1,0.01 ,0 );

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController xbox = new CommandXboxController(0); // My joystick
  private final CommandJoystick joystick = new CommandJoystick(1); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

   private final SwerveRequest.RobotCentric rdrive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1); // Add a 10% deadband

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public final LEDSubsystem led = new LEDSubsystem(0);

  //April Tag T^T
  private Rotation2d m_rotation = new Rotation2d();
  private final Pose2d speakerPosition = new Pose2d(0,0,new Rotation2d(0));

  //SwerveRequest.FieldCentricFacingAngle FieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle();
  FieldCentricFacingAngleFix FieldCentricFacingAngleFix = new FieldCentricFacingAngleFix();


  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(MaxSpeed);


  //intake
  public final IntakeSubsystem intake = new IntakeSubsystem();

  //shooter
  public final ShooterSubsystem shooter = new ShooterSubsystem();

  //Feeder
  public final FeederSubsystem feeder = new FeederSubsystem();

  //pivot
 public final PivotSubsystem pivot = new PivotSubsystem();


  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-xbox.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-xbox.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-xbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    xbox.a().whileTrue(drivetrain.applyRequest(() -> brake));
    xbox.a().whileTrue(led.setRainbowAni((xbox.getLeftX() * 255)));
    xbox.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-xbox.getLeftY(), -xbox.getLeftX()))));

    // reset the field-centric heading on left bumper press
    xbox.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);

    xbox.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    xbox.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
  
    // Limelight data T^T
    var lastResult = LimelightHelpers.getLatestResults("limelight-rear").targetingResults;

    //Math T^T for position
    //Pose2d sdiff = drivetrain.getState().Pose.relativeTo(speakerPosition);
    //Rotate to face April Tag T^T
    xbox.rightTrigger().whileTrue(
      drivetrain.applyRequest(() -> drive.withVelocityX(-xbox.getLeftY() * MaxSpeed)
      .withVelocityY(-xbox.getLeftX() * MaxSpeed)
      .withRotationalRate(turnPID.calculate(LimelightHelpers.getTX("limelight-rear"),0)))
      //.withTargetDirection(new Rotation2d(0))) //Force Set
      //.withTargetDirection(new Rotation2d(drivetrain.getState().Pose.relativeTo(speakerPosition).getRotation().getRadians())))  
      //.withTargetDirection(new Rotation2d(Math.atan(drivetrain.getState().Pose.getY()/drivetrain.getState().Pose.getX())))) //Force Set
      //.withTargetDirection(drivetrain.getPose2d().rotateBy(new Rotation2d(Math.toRadians(LimelightHelpers.getTX("limelight-rear")))).getRotation())) //April Tag T^T
      );
    xbox.rightTrigger().whileTrue(led.setBLUE());

      xbox.x().whileTrue(
      drivetrain.applyRequest(() -> drive.withVelocityX(turnPID.calculate(LimelightHelpers.getTY("limelight-rear"),LimelightHelpers.getTY("limelight-rear")!=0?15:0)* MaxSpeed)
      .withVelocityY(-xbox.getLeftX() * MaxSpeed)
      .withRotationalRate(turnPID.calculate(LimelightHelpers.getTX("limelight-rear"),0)))
      //.withTargetDirection(new Rotation2d(0))) //Force Set
      //.withTargetDirection(new Rotation2d(drivetrain.getState().Pose.relativeTo(speakerPosition).getRotation().getRadians())))  
      //.withTargetDirection(new Rotation2d(Math.atan(drivetrain.getState().Pose.getY()/drivetrain.getState().Pose.getX())))) //Force Set
      //.withTargetDirection(drivetrain.getPose2d().rotateBy(new Rotation2d(Math.toRadians(LimelightHelpers.getTX("limelight-rear")))).getRotation())) //April Tag T^T
      );

    xbox.x().whileTrue(led.setGreen());

    //Robot centric Drive with Bumper (Used with Camera)
    xbox.rightBumper().whileTrue(
      drivetrain.applyRequest(() -> rdrive.withVelocityX(-xbox.getLeftY() * MaxSpeed)
      .withVelocityY(-xbox.getLeftX() * MaxSpeed)
      .withRotationalRate(-xbox.getRightX() * MaxAngularRate)));

    //Align with to SET angle
    xbox.leftTrigger().whileTrue(  
      drivetrain.applyRequest(() -> FieldCentricFacingAngleFix.withVelocityX(-xbox.getLeftY() * MaxSpeed)
      .withVelocityY(-xbox.getLeftX() * MaxSpeed)
      .withTargetDirection(new Rotation2d(0))));


      ///////JOYSTICK CONTROLS
      ////////////////////////
      ////////////////////////

      //intake
      ////////////////////////
      intake.setDefaultCommand(intake.withDisable());
      joystick.button(6).onTrue(intake.slowspeed());
      joystick.button(7).onTrue(intake.midspeed());
      joystick.button(8).onTrue(intake.highspeed());
      joystick.button(9).onTrue(intake.withDisable());

      //Shooter
      shooter.setDefaultCommand(shooter.stop());
      joystick.button(2).onTrue(shooter.ampSpeed());
      joystick.button(3).onTrue(shooter.lowSpeed());
      joystick.button(4).onTrue(shooter.highSpeed());
      joystick.button(5).onTrue(shooter.stop());

      //feeder
      feeder.setDefaultCommand(feeder.withDisable());
      joystick.button(2).onTrue(feeder.slowspeed());
      joystick.button(5).onTrue(feeder.withDisable());

      //pivot
      pivot.setDefaultCommand(pivot.stop());
      joystick.button(2).onTrue(pivot.low());
      joystick.button(5).onTrue(pivot.mid());
  }

  public RobotContainer() {

  configureBindings();

  NamedCommands.registerCommand("setRED", led.setRED());
  NamedCommands.registerCommand("setPink", led.setInit());
  NamedCommands.registerCommand("setBLUE", led.setBLUE());
  NamedCommands.registerCommand("setGREEN", led.setGreen());
  NamedCommands.registerCommand("setRAINBOW", led.setRainbow());
  NamedCommands.registerCommand("setFieldRelative",drivetrain.runOnce(() ->  drivetrain.seedFieldRelative()));
  
  autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
  SmartDashboard.putData("Auto Mode", autoChooser);
  
}

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
    //return runAuto;
  }
}
