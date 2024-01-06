package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;

public class Autos {
    private final DriveSubsystem m_drivetrainSubsystem;
    private SendableChooser<String> autoChooser;
    private HashMap<String, PathPlannerAuto> m_commandMap;
    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");


    public Autos(DriveSubsystem drivetrainSubsystem) 
  {
    m_drivetrainSubsystem = drivetrainSubsystem;
    //AutoConstants.eventMap.put("name of command to run along path", "command itself");
    
    autoChooser = new SendableChooser<>();
    autoTab.add(autoChooser);
    m_commandMap = new HashMap<>();

    autoChooser.addOption("NameOfAutoPath", "NameOfAutoPath");

    m_commandMap.put("NameOfAutoPath", new PathPlannerAuto("NameOfAutoPathBLUE"));
  }



    public Command getAutonomousCommand() {
        String auto = autoChooser.getSelected();
        return m_commandMap.get(auto);
    }
}