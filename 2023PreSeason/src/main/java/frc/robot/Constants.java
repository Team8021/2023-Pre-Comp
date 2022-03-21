package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class Constants {    
    
    private static ShuffleboardTab tab = Shuffleboard.getTab("Constants");
    
    // Shuffleboard variables here
    public static final NetworkTableEntry speedLimit = tab.add("Speed Limit", 1).getEntry();


    // True constants here
    public static final int RIGHT_MOTOR_ID = 5;
}
