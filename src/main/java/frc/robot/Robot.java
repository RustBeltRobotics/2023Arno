package frc.robot;

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
    // private GenericEntry grid01 = matchTab.add("TOP 2", false).withPosition(1, 0).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Purple", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid02 = matchTab.add("TOP 3", false).withPosition(2, 0).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid03 = matchTab.add("TOP 4", false).withPosition(3, 0).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid04 = matchTab.add("TOP 5", false).withPosition(4, 0).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Purple", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid05 = matchTab.add("TOP 6", false).withPosition(5, 0).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid06 = matchTab.add("TOP 7", false).withPosition(6, 0).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid07 = matchTab.add("TOP 8", false).withPosition(7, 0).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Purple", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid08 = matchTab.add("TOP 9", false).withPosition(8, 0).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    private GenericEntry grid10 = matchTab.add("MID 1", false).withPosition(0, 1).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid11 = matchTab.add("MID 2", false).withPosition(1, 1).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Purple", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid12 = matchTab.add("MID 3", false).withPosition(2, 1).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid13 = matchTab.add("MID 4", false).withPosition(3, 1).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid14 = matchTab.add("MID 5", false).withPosition(4, 1).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Purple", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid15 = matchTab.add("MID 6", false).withPosition(5, 1).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid16 = matchTab.add("MID 7", false).withPosition(6, 1).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid17 = matchTab.add("MID 8", false).withPosition(7, 1).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Purple", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid18 = matchTab.add("MID 9", false).withPosition(8, 1).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    private GenericEntry grid20 = matchTab.add("BOT 1", false).withPosition(0, 2).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid21 = matchTab.add("BOT 2", false).withPosition(1, 2).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Purple", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid22 = matchTab.add("BOT 3", false).withPosition(2, 2).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid23 = matchTab.add("BOT 4", false).withPosition(3, 2).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid24 = matchTab.add("BOT 5", false).withPosition(4, 2).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Purple", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid25 = matchTab.add("BOT 6", false).withPosition(5, 2).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid26 = matchTab.add("BOT 7", false).withPosition(6, 2).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid27 = matchTab.add("BOT 8", false).withPosition(7, 2).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Purple", "Color When False", "Red")).withSize(1, 1).getEntry();
    // private GenericEntry grid28 = matchTab.add("BOT 9", false).withPosition(8, 2).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color When True", "Yellow", "Color When False", "Red")).withSize(1, 1).getEntry();

    private GenericEntry timeEntry = matchTab.add("Time Left", 0.).withPosition(1, 3).withWidget(BuiltInWidgets.kTextView).getEntry();

    /**
     * This function is run once when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        // CameraServer.startAutomaticCapture(0);
        // CameraServer.startAutomaticCapture(1);
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

        boolean[][] gridStatus = robotContainer.getGridStatus();
        grid00.setBoolean(gridStatus[0][0]);
        // grid01.setBoolean(gridStatus[0][1]);
        // grid02.setBoolean(gridStatus[0][2]);
        // grid03.setBoolean(gridStatus[0][3]);
        // grid04.setBoolean(gridStatus[0][4]);
        // grid05.setBoolean(gridStatus[0][5]);
        // grid06.setBoolean(gridStatus[0][6]);
        // grid07.setBoolean(gridStatus[0][7]);
        // grid08.setBoolean(gridStatus[0][8]);
        grid10.setBoolean(gridStatus[1][0]);
        // grid11.setBoolean(gridStatus[1][1]);
        // grid12.setBoolean(gridStatus[1][2]);
        // grid13.setBoolean(gridStatus[1][3]);
        // grid14.setBoolean(gridStatus[1][4]);
        // grid15.setBoolean(gridStatus[1][5]);
        // grid16.setBoolean(gridStatus[1][6]);
        // grid17.setBoolean(gridStatus[1][7]);
        // grid18.setBoolean(gridStatus[1][8]);
        grid20.setBoolean(gridStatus[2][0]);
        // grid21.setBoolean(gridStatus[2][1]);
        // grid22.setBoolean(gridStatus[2][2]);
        // grid23.setBoolean(gridStatus[2][3]);
        // grid24.setBoolean(gridStatus[2][4]);
        // grid25.setBoolean(gridStatus[2][5]);
        // grid26.setBoolean(gridStatus[2][6]);
        // grid27.setBoolean(gridStatus[2][7]);
        // grid28.setBoolean(gridStatus[2][8]);

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
        // TODO: Add code to put the robot in coast mode >5 seconds after disabled
        // I think I posted a link to the issue in GitHub related to how we can do this.
        // On second though, this is going to be tough to do since we are not creating
        // our own CANSparkMax objects for the drive train. The arm should be easy
        // enough
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
