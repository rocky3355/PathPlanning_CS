using System;

namespace PathPlanning {

    public class PathPlannerAction {
        public double Cost { get; }
        public int LaneShift { get; }
       // public PathPlannerActionType ActionType { get; }

        public PathPlannerAction(int lane_shift, double cost) {
            Cost = cost;
            LaneShift = lane_shift;
        }
    }
    /*
    public enum PathPlannerActionType {
        None,
        LeftLaneChange,
        RightLaneChange
    }
    */
}
