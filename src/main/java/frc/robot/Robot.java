package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Map;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private Command autonomousCommand;

    private ShuffleboardTab matchTab = Shuffleboard.getTab("Match");

    private GenericEntry grid00 = matchTab.add("TOP 1", false).withPosition(0, 0).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    private GenericEntry grid01 = matchTab.add("TOP 2", false).withPosition(1, 0).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Purple", "Color When False", "Red")).withSize(1, 1).getEntry();
    private GenericEntry grid02 = matchTab.add("TOP 3", false).withPosition(2, 0).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    private GenericEntry grid10 = matchTab.add("MID 1", false).withPosition(0, 1).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    private GenericEntry grid11 = matchTab.add("MID 2", false).withPosition(1, 1).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Purple", "Color When False", "Red")).withSize(1, 1).getEntry();
    private GenericEntry grid12 = matchTab.add("MID 3", false).withPosition(2, 1).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    private GenericEntry grid20 = matchTab.add("BOT 1", false).withPosition(0, 2).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    private GenericEntry grid21 = matchTab.add("BOT 2", false).withPosition(1, 2).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Purple", "Color When False", "Red")).withSize(1, 1).getEntry();
    private GenericEntry grid22 = matchTab.add("BOT 3", false).withPosition(2, 2).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();

    private GenericEntry chunkEntry = matchTab.add("Scoring Chunk", "None!").withPosition(1, 3).withWidget(BuiltInWidgets.kTextView).getEntry();
    private GenericEntry timeEntry = matchTab.add("Time Left", 0.).withPosition(3, 3).withWidget(BuiltInWidgets.kTextView).getEntry();

    /**
     * This function is run once when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        CameraServer.startAutomaticCapture(0);
        CameraServer.startAutomaticCapture(1);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics that you want ran during disabled, autonomous, teleoperated
     * and test.
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        boolean[][] subGridStatus = robotContainer.getSubGridStatus();
        grid00.setBoolean(subGridStatus[0][0]);
        grid01.setBoolean(subGridStatus[0][1]);
        grid02.setBoolean(subGridStatus[0][2]);
        grid10.setBoolean(subGridStatus[1][0]);
        grid11.setBoolean(subGridStatus[1][1]);
        grid12.setBoolean(subGridStatus[1][2]);
        grid20.setBoolean(subGridStatus[2][0]);
        grid21.setBoolean(subGridStatus[2][1]);
        grid22.setBoolean(subGridStatus[2][2]);

        chunkEntry.setString(robotContainer.getChunk());
        timeEntry.setDouble(DriverStation.getMatchTime());
    }

    /**
     * This function is called once at the start of autonomous. It should be used to
     * send the correct autonomous routine to the command scheduler.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
