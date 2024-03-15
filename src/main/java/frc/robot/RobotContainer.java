package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.Taxi;
import frc.robot.autos.Partials.AmpShot;
import frc.robot.autos.Partials.GroundPickup;
import frc.robot.autos.Partials.SpeakerShot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Vision.Pivot;
import frc.robot.commands.Vision.SpeakerAutoArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // NON PathPlanner:
    //private final SendableChooser<Command> m_chooser = new SendableChooser<Command>();

    // Pathplanner:
    private final SendableChooser<Command> m_chooser;

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick aux = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = PS4Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS4Controller.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    private final JoystickButton speed = new JoystickButton(driver, PS4Controller.Button.kR1.value);
    private final JoystickButton aimAtTarget = new JoystickButton(driver, PS4Controller.Button.kCircle.value);
    private final JoystickButton down = new JoystickButton(driver, PS4Controller.Button.kCross.value);
    private final JoystickButton up = new JoystickButton(driver, PS4Controller.Button.kSquare.value);

    /* Aux Buttons */
    private final JoystickButton intakeNote = new JoystickButton(aux, PS4Controller.Button.kL1.value);
    private final JoystickButton outtakeNote = new JoystickButton(aux, PS4Controller.Button.kR1.value);
    private final JoystickButton activateShooter = new JoystickButton(aux, PS4Controller.Button.kR3.value);
    private final JoystickButton AutoArm = new JoystickButton(aux, PS4Controller.Button.kCircle.value);
    private final JoystickButton armGround = new JoystickButton(aux, PS4Controller.Button.kCross.value);
    private final JoystickButton armIdle = new JoystickButton(aux, PS4Controller.Button.kSquare.value);
    private final JoystickButton armAmp = new JoystickButton(aux, PS4Controller.Button.kTriangle.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Limelight s_Limelight = new Limelight();
    private final Arm s_Arm = new Arm();
    private final Intake s_Intake = new Intake(s_Arm);
    private final Climber s_Climber = new Climber();
   
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // (PathPlanner) Registering Named Commands:
        NamedCommands.registerCommand("SpeakerShot", new SpeakerShot(s_Swerve, s_Arm, s_Intake, s_Limelight));
        NamedCommands.registerCommand("AmpShot", new AmpShot(s_Swerve, s_Arm, s_Intake, s_Limelight));
        NamedCommands.registerCommand("GroundPickup", new GroundPickup(s_Swerve, s_Arm, s_Intake, s_Limelight));

        // USB Camera Startup:
        CameraServer.startAutomaticCapture();  

        //Subsystem Defualt Commands:
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(translationAxis), 
                () -> driver.getRawAxis(strafeAxis), 
                () -> driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> speed.getAsBoolean()
            )
        );
        s_Arm.setDefaultCommand(new InstantCommand(()-> s_Arm.stop(), s_Arm));
        s_Intake.setDefaultCommand(new InstantCommand(() -> s_Intake.stopAll(), s_Intake));
        s_Climber.setDefaultCommand(new RunCommand(() -> s_Climber.stop(), s_Climber));

        // (PathPlanner) Build an auto chooser. This will use Commands.none() as the default option.
        m_chooser = AutoBuilder.buildAutoChooser();

        // Adding options to Autonomous Chooser:
        m_chooser.addOption("1 Piece", AutoBuilder.buildAuto("1 Piece"));
        m_chooser.addOption("2 Piece", AutoBuilder.buildAuto("2 Piece"));

        // NON PathPlanner:
        /* m_chooser.setDefaultOption("Defualt", new Taxi(s_Swerve));
        m_chooser.addOption("TEST: SpeakerShot", new SpeakerShot(null, s_Arm, s_Intake, s_Limelight)); */

        //Publish Autonomous Mode Chooser to SmartDashboard
        SmartDashboard.putData("Auto Chooser", m_chooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        aimAtTarget.onTrue(new Pivot(
            s_Limelight, 
            s_Swerve, 
            () -> driver.getRawAxis(translationAxis), 
            () -> driver.getRawAxis(strafeAxis), 
            () -> driver.getRawAxis(rotationAxis), 
            () -> robotCentric.getAsBoolean(),
            () -> speed.getAsBoolean()
            )
        );
        
        aimAtTarget.onFalse(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(translationAxis), 
                () -> driver.getRawAxis(strafeAxis), 
                () -> driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> speed.getAsBoolean()
            )
        );

        up.onTrue(new RunCommand(() -> s_Climber.up(), s_Climber));
        up.onFalse(new RunCommand(() -> s_Climber.stop(), s_Climber));
        down.onTrue(new RunCommand(() -> s_Climber.down(), s_Climber));
        down.onFalse(new RunCommand(() -> s_Climber.stop(), s_Climber));


        /* Aux Buttons */
        intakeNote.onTrue(new RunCommand(()-> s_Intake.intakeNote(), s_Intake));
        intakeNote.onFalse(new RunCommand(()-> s_Intake.stopIntake(), s_Intake));
        outtakeNote.onTrue(new RunCommand(()-> s_Intake.outakeNote(), s_Intake));
        outtakeNote.onFalse(new RunCommand(()-> s_Intake.stopIntake(), s_Intake));
        activateShooter.toggleOnTrue(new RunCommand(()-> s_Intake.shootNote(), s_Intake));

/*         armAmp.onTrue(new RunCommand(() -> s_Arm.forward(), s_Arm));
        armAmp.onFalse(new RunCommand(() -> s_Arm.stop(), s_Arm));

        armGround.onTrue(new RunCommand(() -> s_Arm.backward(), s_Arm));
        armGround.onFalse(new RunCommand(() -> s_Arm.stop(), s_Arm)); */



        AutoArm.onTrue(new SpeakerAutoArm(s_Limelight, s_Arm));
        AutoArm.onFalse(new RunCommand(() -> s_Arm.stop(), s_Arm));
        armGround.onTrue(new RunCommand(() -> s_Arm.setArmPos(Constants.armGroundPOS), s_Arm));
        armIdle.onTrue(new RunCommand(() -> s_Arm.setArmPos(Constants.armIdlePOS), s_Arm));
        armAmp.onTrue(new RunCommand(() -> s_Arm.setArmPos(Constants.armAmpPOS), s_Arm));
   
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}
