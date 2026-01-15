package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class QuickAccessConstants {
        public static final boolean swerveEnabled = true;
        public static final boolean manipulatorsEnabled = true;
        public static final boolean isFieldRelative = true;
        public static final boolean isSoloDrive = true;
    }
    
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
        public static final double kDriveMotorGearRatio = 8.14 / 1.0; // Drive ratio of 8.14 : 1
        public static final double kTurningMotorGearRatio = 1.0 / (150.0 / 7.0); // Turning ratio of (150 / 7) : 1
        public static final double kDriveEncoderRot2Meter = Math.PI * kWheelDiameterMeters / kDriveMotorGearRatio;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
        public static final double kPTurning = 0.5; // For PID
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(22); // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(22); // Distance between front and back wheels

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), // front left
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // front right
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // back left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // back right

        // DRIVE Motor Ports
        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kBackLeftDriveMotorPort = 8;
        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kBackRightDriveMotorPort = 6;

        // TURNING Motor Ports
        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kBackLeftTurningMotorPort = 7;
        public static final int kFrontRightTurningMotorPort = 3;
        public static final int kBackRightTurningMotorPort = 5;

        // CANCoder Ids
        public static final int kFrontLeftCANCoderId = 20;
        public static final int kBackLeftCANCoderId = 23;
        public static final int kFrontRightCANCoderId = 21;
        public static final int kBackRightCANCoderId = 22;


        // Invert booleans | We use MK4i modules so the turning motors are inverted
        public static final boolean kModuleTurningEncoderReversed = true;
        public static final boolean kModuleDriveEncoderReversed = false;
        public static final boolean kModuleCANCoderReversed = false;
        public static final boolean kGyroReversed = false;

        // Turning encoder offsets
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0 * Math.PI / 180;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0 * Math.PI / 180;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0 * Math.PI / 180;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0 * Math.PI / 180;

        // just in case
        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 45.5 * Math.PI / 180;
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -106.5 * Math.PI / 180;
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 88 * Math.PI / 180;
        // public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -119 * Math.PI / 180;

        // Robot speeds
        public static final double kPhysicalMaxSpeedMetersPerSecond = 3.6; // PHYSICAL max speed of the modules (safety cap) 3.6
        public static final double kTeleDriveMaxSpeedMetersPerSecond = 2.5; // Max speed set for teleop

        // Robot turning speeds
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 1.5;

        // Robot acceleration
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        // Robot speed modifiers
        public static final double kTeleopBoostModifier = 1.5;
        public static final double kTeleopSlowModifier = 0.5;
    }

    public static final class OIConstants {

        // Ports
        public static final int kOperatorControllerPort = 0;
        public static final int kDriverControllerPort = 1;
        public static final double kDeadband = 0.04;

        // Joysticks
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2;

        // Buttons
        public static final int kController_x = 1;
        public static final int kController_a = 2;
        public static final int kController_b = 3;
        public static final int kController_y = 4;

        public static final int kController_back = 9;
        public static final int kController_start = 10;
        public static final int kController_leftStickButton = 11;
        public static final int kController_rightStickButton = 12;

        // Triggers [CONTROLLER ONLY]
        public static final int kController_leftBumper = 5;
        public static final int kController_rightBumper = 6;
        public static final int kController_leftTrigger = 7;
        public static final int kController_rightTrigger = 8;

        
        // Flight Sticks
        public static final int kSticks_trigger = 1;
        public static final int kSticks_centerHandle = 2;
        public static final int kSticks_leftHandle = 3;
        public static final int kSticks_rightHandle = 4;

        // Flight Sticks [LEFT HANDED] (from bird's eye view)
        public static final int kLeftSticks_leftGrid_topLeft = 11;
        public static final int kLeftSticks_leftGrid_topMid = 12;
        public static final int kLeftSticks_leftGrid_topRight = 13;
        public static final int kLeftSticks_leftGrid_bottomLeft = 16;
        public static final int kLeftSticks_leftGrid_bottomMid = 15;
        public static final int kLeftSticks_leftGrid_bottomRight = 14;
        public static final int kLeftSticks_rightGrid_topLeft = 7;
        public static final int kLeftSticks_rightGrid_topMid = 6;
        public static final int kLeftSticks_rightGrid_topRight = 5;
        public static final int kLeftSticks_rightGrid_bottomLeft = 8;
        public static final int kLeftSticks_rightGrid_bottomMid = 9;
        public static final int kLeftSticks_rightGrid_bottomRight = 10;

        // Flight Sticks [RIGHT HANDED] (from bird's eye view)
        public static final int kRightSticks_leftGrid_topLeft = 5;
        public static final int kRightSticks_leftGrid_topMid = 6;
        public static final int kRightSticks_leftGrid_topRight = 7;
        public static final int kRightSticks_leftGrid_bottomLeft = 10;
        public static final int kRightSticks_leftGrid_bottomMid = 9;
        public static final int kRightSticks_leftGrid_bottomRight = 8;
        public static final int kRightSticks_rightGrid_topLeft = 13;
        public static final int kRightSticks_rightGrid_topMid = 12;
        public static final int kRightSticks_rightGrid_topRight = 11;
        public static final int kRightSticks_rightGrid_bottomLeft = 14;
        public static final int kRightSticks_rightGrid_bottomMid = 15;
        public static final int kRightSticks_rightGrid_bottomRight = 16;
    } 


    public static final class ElevatorConstants {

        public static final double elevator_maxVelocity = 120; // rotations per second
        public static final double elevator_maxAcceleration = 80; // rotations per second^2
        public static final double elevator_maxVoltage = 8;
        public static final int elevator_leftMotorId = 9;
        public static final int elevator_rightMotorId = 10;

        public static final double elevator_posL4 = 0;
        public static final double elevator_posL3 = 10.3;
        public static final double elevator_posL2 = 5.0;
        public static final double elevator_posL1 = 6.3;
        public static final double elevator_posALGAE3 = 11.0;
        public static final double elevator_posALGAE2 = 5.0;
        public static final double elevator_posSTORE = 0.1;
        public static final double elevator_posSTATION = 4.3;
        
        public static final double m_per_value = 0.07469;

        public static final double elevator_midpointDistanceDifference = 2.5;
        public static final double elevator_placeOffsetDistance = 0;
        public static final double elevator_atPositionTolerance = 0.25;
        public static final double elevator_maxExtention = 12;
        public static final double elevator_minExtention = 0;

        public static final double elevator_kG = 0.03;
        public static final double elevator_kS = 0.01;
        public static final double elevator_kP = 4.5;
        public static final double elevator_kI = 0.0;
        public static final double elevator_kD = 0.3;
    }



    public static final class DifferentialWristConstants {

        public static final int differentialWrist_leftMotorId = 11;
        public static final int differentialWrist_rightMotorId = 12;
        public static final int differentialWrist_articulationEncoderId = 18;
        public static final int differentialWrist_rotationEncoderId = 19;
        public static final int differentialWrist_currentLimit = 60;
        public static final int differentialWrist_atPosTolerance = 7; // In degrees
        
        public static final double differentialWrist_kP = 1.0; // 0.1
        public static final double differentialWrist_kI = 0.0;
        public static final double differentialWrist_kD = 0.02;
        public static final double differentialWrist_period = 0.01; // seconds

        // articulation (degrees), rotation (degrees)
        // Vertical up is 0, downwards is negative

        public static final double[] differentialWrist_posHORIZONTAL = {-120, 0.0};
        public static final double[] differentialWrist_posVERTICAL = {-40, -90};
        public static final double[] differentialWrist_posFLIPPEDVERTICAL = {-45, 90};
        public static final double[] differentialWrist_posMIDPOINT = {-70, -90};
        public static final double[] differentialWrist_posFLIPPEDMIDPOINT = {-70, 90};
        public static final double[] differentialWrist_posALGAE = {-90, 0.0};
        public static final double[] differentialWrist_posSTORE = {0.0, 0.0};
        public static final double[] differentialWrist_posSTATION = {-23.5, 0.0};
        public static final double[] differentialWrist_posEJECT = {-45, 0.0};

        // public static final double[] differentialWrist_posHORIZONTAL = {-2.5, 0.0};
        // public static final double[] differentialWrist_posVERTICAL = {-0.9, 2.2};
        // public static final double[] differentialWrist_posALGAE = {-2.2, 0.0};
        // public static final double[] differentialWrist_posSTORE = {0.0, 0.0};
        // public static final double[] differentialWrist_posSTATION = {-0.4, 0.0};

    }



    public static final class ClawConstants {

        public static final int claw_motorId = 13;
        public static final int claw_laserCanLeftId = 17;
        public static final int claw_laserCanRightId = 16;
        public static final int claw_currentLimit = 55;

        public static final int claw_laserDetectDist = 100;  // In mm
        public static final int claw_laserDetectTolerance = 35; // In mm

        public static final double claw_speedDEFAULT = 0.08;
        public static final double claw_speedSHOOT = 0.12;
        public static final double claw_speedSHOOTSTRONG = 1.0;
        public static final double claw_speedINTAKE = 0.5;
        public static final double claw_speedIDLE = 0.1;
        public static final double claw_speedALGAE = 0.6;
        public static final double claw_speedEJECT = 0.5;
    }



    public static final class ClimbConstants {

        public static final int climb_leftMotorId = 14;
        public static final int climb_rightMotorId = 15;

        public static final double climb_maxPower = 0.21;

        public static final double climb_initialPosition = 50;   // Initial stage
        public static final double climb_finalPosition = 85;     // Final stage

        public static final double climb_initialStageAcceleration = 200;
        public static final double climb_finalStageAcceleration = 50;

        public static final double climb_initialStageVelocity = 1500;
        public static final double climb_finalStageVelocity = 200;

        public static final double climb_kP = 1;
        public static final double climb_kI = 0;
        public static final double climb_kD = 0;
    }



    public static final class LEDConstants {

        public static final double led_colorRainbow = -0.99;
        public static final double led_colorRainbowOcean = -0.95;
        public static final double led_colorStrobeRed = -0.11;
        public static final double led_colorStrobeWhite = -0.05;
        
        public static final double led_colorRed = 0.61;
        public static final double led_colorYellow = 0.69;
        public static final double led_colorGreen = 0.77;
        public static final double led_colorBlue = 0.87;
        public static final double led_colorViolet = 0.91;
        public static final double led_colorWhite = 0.93;
        public static final double led_colorBlack = 0.99;
    }
}