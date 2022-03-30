package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Constants {    
    
    private static ShuffleboardTab tab = Shuffleboard.getTab("Constants");
    

    /** 
     !NOTE: WHEN THE VALUES CHANGE IN SHUFFLEBOARD THEY DO NOT CHANGE IN THE CONSTANTS FILE. 
     THEY ARE "RUNTIME ONLY" BE SURE TO CHANGE THE VALUES AFTER THEY ARE CHANGED IN SHUFFLEBOARD!
    */

     // Shuffleboard variables here
    public static final NetworkTableEntry Speed_Limit = tab.add("Speed Limit", .5)
        .withWidget("Number Slider")
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();


    public static final SendableChooser<SequentialCommandGroup> AutoRoutine = new SendableChooser<>();
    
    // True constants here
    public static final int MOTOR_LEFT_ID = 1;
    public static final int MOTOR_RIGHT_ID = 2;

    public static final double KvLinear = 3;
    public static final double KaLinear = 0.2;
    public static final double KvAngular = .7;
    public static final double KaAngular = .3;
    public static Field2d field = new Field2d();






































    public static final class ControllerConstants
    {
        //controller ids
        public static final int DRIVER_CONTROLLER = 0;
        public static final int OPERATOR_CONTROLLER = 1;

        //joystick ids
        public static final int JOY_X = 0;
        public static final int JOY_Y = 1;
        public static final int JOY_Z = 2;
        public static final int JOY_SLIDE = 3;
        public static final int TRIGGER = 1;
        public static final int THUMB_BUTTON = 2;
        public static final int TOP_DOWN_LEFT = 3;
        public static final int TOP_DOWN_RIGHT = 4;
        public static final int TOP_UP_LEFT = 5;
        public static final int TOP_UP_RIGHT = 6;
        public static final int PAD_A1 = 7;
        public static final int PAD_A2 = 8;
        public static final int PAD_B1 = 9;
        public static final int PAD_B2 = 10;
        public static final int PAD_C1 = 11;
        public static final int PAD_C2 = 12;

        //xbox ids
        public static final int LEFT_X = 0;
        public static final int LEFT_Y = 1;
        public static final int LEFT_BUMPER_AXIS = 2;
        public static final int RIGHT_BUMPER_AXIS = 3;
        public static final int RIGHT_X = 4;
        public static final int RIGHT_Y = 5;
        public static final int X_BUTTON = 1;
        public static final int A_BUTTON = 2;
        public static final int B_BUTTON = 3;
        public static final int Y_BUTTON = 4;
        public static final int LEFT_BUMPER = 5;
        public static final int RIGHT_BUMPER = 6;
        public static final int LEFT_TRIGGER = 7;
        public static final int RIGHT_TRIGGER = 8;
        public static final int BACK_BUTTON = 9;
        public static final int START_BUTTON = 10;
        public static final int LEFT_JOY_CLICK = 11;
        public static final int RIGHT_JOY_CLICK = 12;

        //pov ids
        public static final int POV_N = 0;
        public static final int POV_NE = 45;
        public static final int POV_E = 90;
        public static final int POV_SE = 135;
        public static final int POV_S = 180;
        public static final int POV_SW = 225;
        public static final int POV_W = 270;
        public static final int POV_NW = 315;
    }

}
