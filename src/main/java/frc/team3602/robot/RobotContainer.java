package frc.team3602.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import frc.team3602.robot.subsystems.drive.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.drive.generated.TunerConstants;

public class RobotContainer {
    /* Controllers */
    private CommandXboxController xbox = new CommandXboxController(0);
    private CommandXboxController xbox2 = new CommandXboxController(1);

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Subsystems */
    private DrivetrainSubsystem driveSubsys = TunerConstants.createDrivetrain();
    private Vision vision = new Vision();

    /* Sendable Choosers */
    public SendableChooser<Command> autoChooser;
    public SendableChooser<Double> polarityChooserForward;
    public SendableChooser<Double> polarityChooserSideways;

    public RobotContainer() {
        if (RobotBase.isSimulation()) {
            simConfigDefaultCommands();
            simConfigButtonBindings();
            vision.resetVisSim();
        } else {
            configDefaultCommands();
            configButtonBindings();
        }

        configNamedCommands();
        driveSubsys.configAutoBuilder();
        configSendableChoosers();
        driveSubsys.registerTelemetry(logger::telemeterize);
    }

    private void simConfigDefaultCommands() {
        driveSubsys.setDefaultCommand(driveSubsys.applyRequest(() -> drive
                .withVelocityX(-xbox.getLeftX()
                        * 0.8 * MaxSpeed * polarityChooserForward.getSelected())
                .withVelocityY(xbox.getLeftY() *
                        0.8 * MaxSpeed * polarityChooserSideways.getSelected())
                .withRotationalRate(xbox2.getLeftY() * 3.2 * MaxAngularRate)));
    }

    private void configDefaultCommands() {
        driveSubsys.setDefaultCommand(driveSubsys.applyRequest(() -> drive
                .withVelocityX(-xbox.getLeftY() * polarityChooserForward.getSelected()
                        * 0.8 * MaxSpeed) // Drive
                .withVelocityY(-xbox.getLeftX() * polarityChooserSideways.getSelected() * 0.8 * MaxSpeed)
                .withRotationalRate(-xbox.getRightX() * 3.2 * MaxAngularRate)));
    }

    private void simConfigButtonBindings() {

    }

    private void configButtonBindings() {

    }

    /**
     * Registers commands, allows us to refernce in pathplanner for autons
     * MUST be called BEFORE configuring the AutoBuilder
     */
    private void configNamedCommands() {
        NamedCommands.registerCommand("Test", print("Testing pathplanner"));
    }

    /**
     * Assigns values to our sendable choosers and puts them on Smart Dashboard,
     * MUST be called AFTER configuring the AutoBuilder
     */
    private void configSendableChoosers() {
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("Null", print("NO AUTON SELECTED (you are a buffoon!!)"));
        SmartDashboard.putData("Auto Chooser", autoChooser);

        polarityChooserForward = new SendableChooser<>();
        polarityChooserForward.setDefaultOption("Standard", 1.0);
        polarityChooserForward.addOption("Inverted", -1.0);
        SmartDashboard.putData("Forward Polarity Chooser", polarityChooserForward);

        polarityChooserSideways = new SendableChooser<>();
        polarityChooserSideways.setDefaultOption("Standard", 1.0);
        polarityChooserSideways.addOption("Inverted", -1.0);
        SmartDashboard.putData("Sideways Polarity Chooser", polarityChooserSideways);
    }

    public void updateVisionSim() {
        vision.updateVisSim(driveSubsys.getState().Pose);
    }

    public void updateVision() {
        vision.updateReadings();
    }

    public void updateRobotPose_Vision() {
        //if (vision.target1Visible) {
            driveSubsys.addVisionMeasurement(vision.camera1EstPose, vision.camera1PETimeStamp);
        //}

        if (vision.target2Visible) {
            driveSubsys.addVisionMeasurement(vision.camera1EstPose, vision.camera1PETimeStamp);
        }
    }
}
