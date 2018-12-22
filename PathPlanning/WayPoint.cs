using System;

namespace PathPlanning {

    public class Vector2 {
        public double x { get; }
        public double y { get; }

        public Vector2(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    public class WayPoint {

        public double x { get; }
        public double y { get; }
        public double s { get; }
        public Vector2 d { get; }

        public WayPoint(double x, double y, double s, Vector2 d) {
            this.x = x;
            this.y = y;
            this.s = s;
            this.d = d;
        }
    }
}
