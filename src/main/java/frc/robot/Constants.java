package frc.robot;

public class Constants{
    public static final int safetyDiv = 4;

    public static class PS4ID{
        /* BUTTON IDs (PS4) */
        public static final int square = 1;
        public static final int cross = 2;
        public static final int circle = 3;
        public static final int triangle = 4;
        public static final int l1 = 5;
        public static final int r1 = 6;
        public static final int l2b = 7;
        public static final int r2b = 8;
        public static final int share = 9;
        public static final int option = 10;
        public static final int l3b = 11;
        public static final int r3b = 12;
        public static final int ps = 13;
        public static final int touchpad = 14;

        /* AXIS IDs (PS4) */
        public static final int l3h = 0;
        public static final int l3v = 1;
        public static final int r3h = 2;
        public static final int l2a = 3;
        public static final int r2a = 4;
        public static final int r3v = 5;
    }

    public static class CAN {
        /* DRIVETRAIN */
        public static final int driveRightMaster = 1;
        public static final int driveRightSlave = 2;
        public static final int driveLeftMaster = 3;
        public static final int driveLeftSlave = 4;

        /* TURRET */
        public static final int turretFly = 9;
        public static final int turretAim = 7;
        public static final int turretLoad = 6;

        /* INTAKE */
        public static final int intakeBelt = 11;
    }

    public static class JOYSTICKS{
        public static final int xbox = 0;
        public static final int ps4 = 1;
    }

    public static class VISION{
        // table name for camera
        public static final String cameraName = "TurretCam";
    }

    /* PUBLIC METHODS BELOW */
    public static double expScale(double input) {
        if (input < 0) {
         return -Math.pow(2, -input) + 1;
        } else {
           return Math.pow(2, input) - 1;
        }
    }

    public static double linScale(double input) {
        return (input * 0.5) + 0.5;
    }
}