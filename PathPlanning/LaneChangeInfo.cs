using System;

namespace PathPlanning {

    public class LaneChangeInfo {

        public double AdjacentCarDeltaSpeed { get; }

        public LaneChangeInfo(double adjacentCarDeltaSpeed) {
            AdjacentCarDeltaSpeed = adjacentCarDeltaSpeed;
        }
    }
}
