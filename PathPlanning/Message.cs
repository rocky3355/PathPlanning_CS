using System;

namespace PathPlanning {

    public class Message {
        public double x;
        public double y;
        public double s;
        public double d;
        public double yaw;
        public double speed;
        public double[] previous_path_x;
        public double[] previous_path_y;
    }
}
