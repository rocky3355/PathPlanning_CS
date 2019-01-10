using System;
using System.Linq;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using Fleck;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace PathPlanning {

    public static class Program {

        // Consider double lane changes
        // Consider speed of adjacent cars to predict their position in the future if car behind is faster than us


        private const double LANE_COST = 3.0;
        private const double SPEED_COST = 1.0;
        private const double BRAKE_SPEED_FACTOR = 0.01;
        private const double BRAKE_DIST_FACTOR = 0.002;
        private const int NUMBER_OF_ROUGH_POINTS = 5;
        private const double TIME_TO_REACH_POINT = 0.02;
        private const double MS_TO_MPH = 2.237;
        private const double MPH_TO_MS = 1.0 / MS_TO_MPH;
        private const int NUMBER_OF_PATH_POINTS = 50;
        private const double ROUGH_STEP_SIZE = 30.0;
        private const double TARGET_VELOCITY_MPH = 49.5;
        private const double LANE_WIDTH = 4.0;
        private const double MAX_SPEED_DIFF = 0.2;
        private const double MIN_FREE_SPACE_LANE_CHANGE = 10.0;
        private const double TIME_UNTIL_NEXT_LANE_CHANGE_MS = 5000;
        private const double ADJECENT_MAX_DISTANCE_FOR_CONSIDERATION = 60.0;
        private const double LANE_CHANGE_ROUGH_STEP_SIZE_FACTOR = 1.7;
        private const int DESIRED_LANE = 1;
        private const int MIN_LANE = 0;
        private const int MAX_LANE = 2;

        private static List<WayPoint> map_points;
        private static IWebSocketConnection _socket;

        private static int lane = 1;
        private static int last_lane_change = 0;
        private static double desired_velocity_mph = 0;
        private static PathPlannerAction current_action = null;

        public static void Main(string[] args) {
            LoadMap();
            StartServer();
        }

        private static void LoadMap() {
            map_points = new List<WayPoint>();
            using (StreamReader reader = new StreamReader("../../../Data/highway_map.csv")) {
                while (!reader.EndOfStream) {
                    string line = reader.ReadLine();
                    string[] parts = line.Split(' ');
                    double x = ToDouble(parts[0]);
                    double y = ToDouble(parts[1]);
                    double s = ToDouble(parts[2]);
                    double d_x = ToDouble(parts[3]);
                    double d_y = ToDouble(parts[4]);
                    map_points.Add(new WayPoint(x, y, s, new Vector2(d_x, d_y)));
                }
            }
        }

        private static void StartServer() {
            WebSocketServer server = new WebSocketServer("ws://127.0.0.1:4567");
            server.Start(socket => {
                _socket = socket;
                socket.OnOpen = () => Console.WriteLine("Connected!");
                socket.OnClose = () => Console.WriteLine("Disconnected!");
                socket.OnMessage = message => OnMessage(message);
            });

            while (true) {
                Thread.Sleep(1000);
            }
        }

        private static string HasData(string message) {
            if (message.Contains("null") || !message.Contains("telemetry")) {
                return null;
            }

            int b1 = message.IndexOf("[");
            int b2 = message.IndexOf("}");
            if (b1 == -1 || b2 == -1) {
                return null;
            }

            message = message.Substring(b1, b2 - b1 + 1);
            message = message.Replace("[\"telemetry\",", "");
            return message;
        }

        private static LaneChangeInfo CheckCarsOnAdjacentLane(int lane_shift, double ego_s, double[][] sensor_fusion) {
            int target_lane = lane + lane_shift;
            if (target_lane < MIN_LANE || target_lane > MAX_LANE) {
                return null;
            }

            int nearest_car_idx = -1;
            double lowest_distance = ADJECENT_MAX_DISTANCE_FOR_CONSIDERATION;

            for (int i = 0; i < sensor_fusion.Length; i++) {
                double[] car_data = sensor_fusion[i];

                double s = car_data[5];
                double d = car_data[6];
                double longitudinal_distance = s - ego_s;

                if (d > target_lane * LANE_WIDTH && d < (target_lane + 1) * LANE_WIDTH) {
                    double free_space = Math.Abs(longitudinal_distance);
                    if (free_space < MIN_FREE_SPACE_LANE_CHANGE) {
                        return null;
                    }

                    if (longitudinal_distance > 0 && longitudinal_distance < lowest_distance) {
                        nearest_car_idx = i;
                        lowest_distance = longitudinal_distance;
                    }
                }
            }

            double delta_v = 0;
            if (nearest_car_idx > -1) {
                double vx = sensor_fusion[nearest_car_idx][3];
                double vy = sensor_fusion[nearest_car_idx][4];
                double v = Math.Sqrt(vx * vx + vy * vy) * MS_TO_MPH;
                delta_v = TARGET_VELOCITY_MPH - v;
            }

            LaneChangeInfo info = new LaneChangeInfo(delta_v);
            return info;
        }

        private static double GetCenterOfLane(int lane) {
            double center = lane * LANE_WIDTH + LANE_WIDTH / 2;
            return center;
        }

        private static void OnMessage(string message) {
            message = HasData(message);

            if (message == null) {
                _socket.Send("42[\"manual\",{}]");
                return;
            }

            Message msg = JsonConvert.DeserializeObject<Message>(message);

            double brake_speed = 0;
            bool car_in_front = false;

            for (int i = 0; i < msg.sensor_fusion.Length; i++) {
                double[] car_data = msg.sensor_fusion[i];
                double vx = car_data[3];
                double vy = car_data[4];
                double s = car_data[5];
                double d = car_data[6];

                if (d > lane * LANE_WIDTH && d < (lane + 1) * LANE_WIDTH) {
                    double distance = s - msg.s;
                    double distance_threshold = msg.speed / 1.2;

                    if (s > msg.s && distance < distance_threshold) {
                        double distance_violation = distance_threshold - distance;
                        double car_vel_mph = Math.Sqrt(vx * vx + vy * vy) * MS_TO_MPH;
                        double speed_diff = desired_velocity_mph - car_vel_mph;
                        brake_speed = speed_diff * BRAKE_SPEED_FACTOR + distance_violation * BRAKE_DIST_FACTOR;
                        car_in_front = true;

                        break;
                    }
                }
            }

            if (car_in_front) {
                brake_speed = brake_speed > MAX_SPEED_DIFF ? MAX_SPEED_DIFF : brake_speed;
                desired_velocity_mph -= brake_speed;
            }
            // Don't use msg.speed, it lags behind and will cause desired velocity to get too high
            else if (desired_velocity_mph < TARGET_VELOCITY_MPH) {
                desired_velocity_mph += MAX_SPEED_DIFF;
            }

            if (current_action == null) {
                LaneChangeInfo info_left = CheckCarsOnAdjacentLane(-1, msg.s, msg.sensor_fusion);
                LaneChangeInfo info_right = CheckCarsOnAdjacentLane(1, msg.s, msg.sensor_fusion);

                List<PathPlannerAction> actions = new List<PathPlannerAction>();
                double straight_cost = Math.Abs(TARGET_VELOCITY_MPH - desired_velocity_mph) * SPEED_COST + Math.Abs(lane - DESIRED_LANE) * LANE_COST;
                actions.Add(new PathPlannerAction(0, straight_cost));

                // TODO: Use timer
                int time_since_last_last_change = Environment.TickCount - last_lane_change;

                if (time_since_last_last_change > TIME_UNTIL_NEXT_LANE_CHANGE_MS && (lane != DESIRED_LANE || car_in_front)) {
                    if (info_left != null) {
                        double lane_change_left_cost = Math.Abs((lane - 1) - DESIRED_LANE) * LANE_COST + info_left.AdjacentCarDeltaSpeed * SPEED_COST;
                        actions.Add(new PathPlannerAction(-1, lane_change_left_cost));
                    }
                    if (info_right != null) {
                        double lane_change_right_cost = Math.Abs((lane + 1) - DESIRED_LANE) * LANE_COST + info_right.AdjacentCarDeltaSpeed * SPEED_COST;
                        actions.Add(new PathPlannerAction(1, lane_change_right_cost));
                    }
                }

                // Round the costs to filter out jitters. If values are the same, straight will be preferred
                // as this action is the first one in the list
                PathPlannerAction desired_action = actions.OrderBy(a => Math.Round(a.Cost)).First();
                lane += desired_action.LaneShift;
                if (desired_action.LaneShift != 0) {
                    current_action = desired_action;
                }
            }
            else {
                if (msg.d > lane * LANE_WIDTH && msg.d < (lane + 1) * LANE_WIDTH) {
                    current_action = null;
                    last_lane_change = Environment.TickCount;
                }
            }

            double desired_velocity_ms = desired_velocity_mph * MPH_TO_MS;

            double smooth_step = desired_velocity_ms * TIME_TO_REACH_POINT;
            double[] rough_x = new double[NUMBER_OF_ROUGH_POINTS];
            double[] rough_y = new double[NUMBER_OF_ROUGH_POINTS];
            int prev_size = msg.previous_path_x.Length;

            double ref_x = msg.x;
            double ref_y = msg.y;
            double ref_yaw = deg2rad(msg.yaw);

            if (prev_size < 2) {
                double prev_car_x = msg.x - Math.Cos(msg.yaw);
                double prev_car_y = msg.y - Math.Sin(msg.yaw);
                rough_x[0] = prev_car_x;
                rough_x[1] = msg.x;
                rough_y[0] = prev_car_y;
                rough_y[1] = msg.y;
            }
            else {
                ref_x = msg.previous_path_x[prev_size - 1];
                ref_y = msg.previous_path_y[prev_size - 1];
                double ref_x_prev = msg.previous_path_x[prev_size - 2];
                double ref_y_prev = msg.previous_path_y[prev_size - 2];
                ref_yaw = Math.Atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                rough_x[0] = ref_x_prev;
                rough_x[1] = ref_x;
                rough_y[0] = ref_y_prev;
                rough_y[1] = ref_y;
            }

            for (int i = 2; i < NUMBER_OF_ROUGH_POINTS; i++) {
                double next_s = msg.s + (i - 1) * ROUGH_STEP_SIZE * (current_action == null ? 1.0 : LANE_CHANGE_ROUGH_STEP_SIZE_FACTOR);
                double next_d = GetCenterOfLane(lane);
                double[] xy = getXY(next_s, next_d);
                rough_x[i] = xy[0];
                rough_y[i] = xy[1];
            }

            double yaw_sin = Math.Sin(-ref_yaw);
            double yaw_cos = Math.Cos(-ref_yaw);

            for (int i = 0; i < NUMBER_OF_ROUGH_POINTS; i++) {
                double shift_x = rough_x[i] - ref_x;
                double shift_y = rough_y[i] - ref_y;
                rough_x[i] = shift_x * yaw_cos - shift_y * yaw_sin;
                rough_y[i] = shift_x * yaw_sin + shift_y * yaw_cos;
            }

            CubicSpline spline = new CubicSpline(rough_x, rough_y);

            double[] next_x_vals = new double[NUMBER_OF_PATH_POINTS];
            double[] next_y_vals = new double[NUMBER_OF_PATH_POINTS];

            for (int i = 0; i < prev_size; i++) {
                next_x_vals[i] = msg.previous_path_x[i];
                next_y_vals[i] = msg.previous_path_y[i];
            }

            yaw_sin = Math.Sin(ref_yaw);
            yaw_cos = Math.Cos(ref_yaw);

            for (int i = 0; i < next_x_vals.Length - prev_size; i++) {
                double x_point = (i + 1) * smooth_step;
                double y_point = spline.Eval(new double[] { x_point })[0];

                double x_ref = x_point;

                x_point = x_ref * yaw_cos - y_point * yaw_sin;
                y_point = x_ref * yaw_sin + y_point * yaw_cos;
                x_point += ref_x;
                y_point += ref_y;

                next_x_vals[prev_size + i] = x_point;
                next_y_vals[prev_size + i] = y_point;
            }     

            JObject answer = new JObject();
            answer.Add("next_x", JToken.FromObject(next_x_vals));
            answer.Add("next_y", JToken.FromObject(next_y_vals));
            string answer_str = "42[\"control\"," + answer.ToString() + "]";

            _socket.Send(answer_str);
        }

        private static double ToDouble(string str) {
            return Double.Parse(str, System.Globalization.CultureInfo.InvariantCulture);
        }

        private static double deg2rad(double x) {
            return x * Math.PI / 180.0;
        }

        private static double rad2deg(double x) {
            return x * 180.0 / Math.PI;
        }

        private static double[] getXY(double s, double d) {
	        int prev_wp = -1;

	        while(s > map_points[prev_wp + 1].s && (prev_wp < map_points.Count - 1)) {
		        prev_wp++;
	        }

            int wp2 = (prev_wp + 1) % map_points.Count;

            double heading = Math.Atan2(map_points[wp2].y - map_points[prev_wp].y, map_points[wp2].x - map_points[prev_wp].x);
            // the x,y,s along the segment
            double seg_s = s - map_points[prev_wp].s;

            double seg_x = map_points[prev_wp].x + seg_s * Math.Cos(heading);
            double seg_y = map_points[prev_wp].y + seg_s * Math.Sin(heading);

            double perp_heading = heading - Math.PI / 2;

            double x = seg_x + d * Math.Cos(perp_heading);
            double y = seg_y + d * Math.Sin(perp_heading);

            return new double[] { x, y };
        }

        private static double distance(double x1, double y1, double x2, double y2) {
            return Math.Sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
        }

        private static int ClosestWaypoint(double x, double y) {
            int closestWaypoint = 0;
            double closestLen = Double.MaxValue;

	        for (int i = 0; i < map_points.Count; i++) {
		        double map_x = map_points[i].x;
                double map_y = map_points[i].y;
                double dist = distance(x, y, map_x, map_y);
		        if(dist<closestLen) {
			        closestLen = dist;
			        closestWaypoint = i;
		        }
            }
	        return closestWaypoint;
        }

        private static int NextWaypoint(double x, double y, double theta) {
            int closestWaypoint = ClosestWaypoint(x, y);

            double map_x = map_points[closestWaypoint].x;
            double map_y = map_points[closestWaypoint].y;
            double heading = Math.Atan2((map_y - y), (map_x - x));

            double angle = Math.Abs(theta - heading);
            angle = Math.Min(2 * Math.PI - angle, angle);

            if (angle > Math.PI / 4) {
                closestWaypoint++;
                if (closestWaypoint == map_points.Count) {
                    closestWaypoint = 0;
                }
            }
            return closestWaypoint;
        }

        // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
        private static double[] getFrenet(double x, double y, double theta) {
            int next_wp = NextWaypoint(x, y, theta);

            int prev_wp;
            prev_wp = next_wp - 1;
            if (next_wp == 0) {
                prev_wp = map_points.Count - 1;
            }

            double n_x = map_points[next_wp].x - map_points[prev_wp].x;
            double n_y = map_points[next_wp].y - map_points[prev_wp].y;
            double x_x = x - map_points[prev_wp].x;
            double x_y = y - map_points[prev_wp].y;

            // find the projection of x onto n
            double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
            double proj_x = proj_norm * n_x;
            double proj_y = proj_norm * n_y;

            double frenet_d = distance(x_x, x_y, proj_x, proj_y);

            //see if d value is positive or negative by comparing it to a center point

            double center_x = 1000 - map_points[prev_wp].x;
            double center_y = 2000 - map_points[prev_wp].y;
            double centerToPos = distance(center_x, center_y, x_x, x_y);
            double centerToRef = distance(center_x, center_y, proj_x, proj_y);

            if (centerToPos <= centerToRef) {
                frenet_d *= -1;
            }

            // calculate s value
            double frenet_s = 0;
            for (int i = 0; i < prev_wp; i++) {
                frenet_s += distance(map_points[i].x, map_points[i].y, map_points[i + 1].x, map_points[i + 1].y);
            }

            frenet_s += distance(0, 0, proj_x, proj_y);
            return new double[] { frenet_s, frenet_d };
        }
    }
}
