package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.superstructure.BallGrabber;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.SubsystemManager;
import frc.robot.subsystems.superstructure.Arm;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.BallGrabberCommand;
import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import swervelib.SwerveInputStream;
import frc.robot.commands.SubsystemManagerCommand;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autonomous.Auto;
import frc.robot.autonomous.AutoCommands;
import frc.robot.autonomous.AutoNames;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


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
    private final Elevator elevator = new Elevator();
    private final Arm arm = new Arm();
    private final BallGrabber ballGrabber = new BallGrabber();
    private final SubsystemManager subsystemManager = new SubsystemManager(drivebase, elevator, arm, ballGrabber, controls);

    private final AutoCommands factory;
    private final SendableChooser<Auto> autoChooser;

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);


    public RobotContainer() {
      arm.setDefaultCommand(new ArmCommand(arm, controls));

      elevator.setDefaultCommand(new ElevatorCommand(elevator,controls));
      ballGrabber.setDefaultCommand(new BallGrabberCommand(ballGrabber, controls));
      subsystemManager.setDefaultCommand(new SubsystemManagerCommand(drivebase, elevator, arm, ballGrabber, controls, subsystemManager));
      
      factory = new AutoCommands(
      null,
      drivebase
      );

      autoChooser = new SendableChooser<Auto>();

      for(AutoNames auto : AutoNames.values()) {
        Auto autonomousRoutine = new Auto(auto, factory);
        if(auto == AutoNames.TEST){
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
        driverXbox.y().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
        driverXbox.x().onTrue(Commands.defer(() -> drivebase.alignToTrenchCommand(), Set.of(drivebase)));
        driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));
        driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    }

    public Command getAutonomousCommand() {
      return autoChooser.getSelected().getCommand();
    }
}
