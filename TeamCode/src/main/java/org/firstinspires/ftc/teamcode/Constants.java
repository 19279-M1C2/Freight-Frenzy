//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.amarcolini.joos.control.FeedforwardCoefficients;
//import com.amarcolini.joos.control.PIDCoefficients;
//import com.amarcolini.joos.trajectory.config.GenericConstraints;
//
//@Config
//public class Constants {
//    static double ULTRAPLANATARY_TICKS = 28;
//    static double ULTRAPLANATARY_DMAX_RPM = 6000.0;
//
//    static String DRIVE_LEFT_A_NAME = "drive-left-a";
//    static String DRIVE_LEFT_B_NAME = "drive-left-b";
//    static String DRIVE_RIGHT_A_NAME = "drive-right-a";
//    static String DRIVE_RIGHT_B_NAME = "drive-right-b";
//
//    static class Module {
//        static double GEAR_RATIO = 1.0 / 30.0;
//        static double TICKS_PER_REV = 100.0;
//        static double TRACK_WIDTH = 16.0;
//        static double WHEEL_RADIUS = 2.0;
//    }
//
//    static class Coefficients {
//        static PIDCoefficients MODULE_PID = new PIDCoefficients(4.0, 0.0, 0.1);
//        static FeedforwardCoefficients FEEDFORWARD_COEFFICIENTS = new FeedforwardCoefficients();
//        static GenericConstraints TRAJECTORY_CONSTRAINTS = new GenericConstraints();
//        static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(1.0, 0.0, 0.5);
//        static PIDCoefficients HEADING_PID = new PIDCoefficients(1.0, 0.0, 0.5);
//    }
//}
