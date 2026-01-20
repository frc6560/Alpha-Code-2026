package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.SotmCommands;
import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.superstructure.Flywheel;
// import frc.robot.subsystems.superstructure.Hood;
import frc.robot.subsystems.superstructure.ShooterLUT;
// import frc.robot.subsystems.superstructure.Turret;
import frc.robot.subsystems.superstructure.Flywheel;
import frc.robot.subsystems.superstructure.Feeder;
// import frc.robot.subsystems.superstructure.Intake;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import swervelib.SwerveInputStream;
import frc.robot.subsystems.superstructure.Sotm;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autonomous.Auto;
import frc.robot.autonomous.AutoFactory;
import frc.robot.autonomous.AutoRoutines;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVision;


public class RobotContainer {

    // Controllers
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final XboxController firstXbox = new XboxController(0);
    private final XboxController secondXbox = new XboxController(1);
    private final ManualControls controls = new ManualControls(firstXbox, secondXbox);

     // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
    "swerve/falcon"));
    private final VisionSubsystem vision;
    
    // Subsystems
    private final Flywheel flywheel = new Flywheel();
    // private final Turret turret = new Turret();
    private final ShooterLUT shooterLUT = new ShooterLUT();
    // private final Hood hood = new Hood();
    private final Feeder feeder = new Feeder();
    // private final Intake intake = new Intake();
    private final Sotm sotm = new Sotm(drivebase, flywheel, controls, shooterLUT, feeder /*hood, turret, intake*/);
    


    private final AutoFactory factory;
    private final SendableChooser<Auto> autoChooser;

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);


    public RobotContainer() {
      sotm.setDefaultCommand(new SotmCommands(sotm, drivebase, flywheel, controls, shooterLUT/*, hood, turret*/));

      factory = new AutoFactory(
      null,
      drivebase
      );

      autoChooser = new SendableChooser<Auto>();

      for(AutoRoutines auto : AutoRoutines.values()) {
        Auto autonomousRoutine = new Auto(auto, factory);
        if(auto == AutoRoutines.TEST){
          autoChooser.setDefaultOption(autonomousRoutine.getName(), autonomousRoutine);
        }
        else {
          autoChooser.addOption(autonomousRoutine.getName(), autonomousRoutine);
        }
      }

      List<LimelightVision> limelights = new ArrayList<LimelightVision>();
      for(String name : LimelightConstants.LIMELIGHT_NAMES) {
        Pose3d cameraPose = LimelightConstants.getLimelightPose(name);
        limelights.add(new LimelightVision(drivebase, name, cameraPose));
      }

      vision = new VisionSubsystem(limelights);
      configureBindings();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        driverXbox.a().onTrue(
          Commands.defer(() -> {
            return Commands.runOnce(() -> vision.hardReset("limelight"), vision);
          }, Set.of(vision))
        );
        
        driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));

        new Trigger(controls::BallOut).whileTrue(
                Commands.runEnd(
                    sotm::feed,   // Run while held
                    sotm::stop,   // Stop when released
                    sotm
                )
            );

        new Trigger(controls::runIntake).whileTrue(
                Commands.runEnd(
                    sotm::runIntake,   // Run while held
                    sotm::stopIntake,   // Stop when released
                    sotm
                )
            );

    }

    public Command getAutonomousCommand() {
      return autoChooser.getSelected().getCommand();
    }
}
