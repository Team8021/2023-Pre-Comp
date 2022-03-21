package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class Constants {    
    
    private static ShuffleboardTab tab = Shuffleboard.getTab("Constants");
    
    
    public static final NetworkTableEntry speedLimit = tab.add("Speed Limit", 1).getEntry();

    // public static double speedLimit = tab.add("Speed Limit", 8).getEntry().getDouble(0);
    
    



    

    // Runtime variables Down here
    


}
