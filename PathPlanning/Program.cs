using System;
using System.Linq;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using Fleck;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace PathPlanning {

    public class Program {

        private static List<WayPoint> map_points;
        private static IWebSocketConnection _socket;

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
                socket.OnOpen = () => Console.WriteLine("Open!");
                socket.OnClose = () => Console.WriteLine("Close!");
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

        private static void OnMessage(string message) {
            message = HasData(message);

            if (message == null) {
                _socket.Send("42[\"manual\",{}]");
                return;
            }

            Message msg = JsonConvert.DeserializeObject<Message>(message);

            // TODO: Make this depending on planning distance?
            const int NUMBER_OF_ROUGH_POINTS = 3;
            const double TIME_TO_REACH_POINT = 0.02;

            double planning_distance = 30.0; // Math.Max(msg.speed, 10.0);
            double desired_velocity_mph = 49.5;

            double vel_diff_mph = desired_velocity_mph - msg.speed;
            if (vel_diff_mph > 0.5 || vel_diff_mph < -0.5) {
                desired_velocity_mph = msg.speed + vel_diff_mph / 50.0;
            }

            //Console.WriteLine(msg.speed + " /// " + desired_velocity_mph + " /// " + vel_diff_mph);
            double desired_velocity_ms = desired_velocity_mph / 2.237;

            double smooth_step = desired_velocity_ms * TIME_TO_REACH_POINT;
            double[] rough_x = new double[NUMBER_OF_ROUGH_POINTS];
            double[] rough_y = new double[NUMBER_OF_ROUGH_POINTS];

            int rough_idx = 0;
            int prev_size = msg.previous_path_x.Length;
            double start_s = msg.s + smooth_step;

            // TODO: remove
            double[] xy = null;

            // TODO: Use one or two last points?
            if (prev_size > 0) {
                rough_x[rough_idx] = msg.previous_path_x[0];
                rough_y[rough_idx++] = msg.previous_path_y[0];
                double[] sd = getFrenet(rough_x[0], rough_y[0], deg2rad(msg.yaw));
                start_s = sd[0];
                //rough_x[rough_idx] = msg.previous_path_x[1];
                //rough_y[rough_idx++] = msg.previous_path_y[1];
            }
            else {
                xy = getXY(start_s, msg.d);
                rough_x[rough_idx] = xy[0];
                rough_y[rough_idx++] = xy[1];
            }

            double rough_step = planning_distance / NUMBER_OF_ROUGH_POINTS;
            for (int i = rough_idx; i < NUMBER_OF_ROUGH_POINTS; i++) {
                double next_s = start_s + i * rough_step;
                double next_d = 6.0;
                xy = getXY(next_s, next_d);
                rough_x[i] = xy[0];
                rough_y[i] = xy[1];
            }

            CubicSpline spline = new CubicSpline(rough_x, rough_y);

            double path_length = rough_x.Last() - rough_x.First();
            int steps = (int)Math.Round(path_length / smooth_step);
            double[] next_x_vals = new double[steps];

            for (int i = 0; i < steps; i++) {
                double x = rough_x[0] + i * smooth_step;
                next_x_vals[i] = x;
            }

            double[] next_y_vals = spline.Eval(next_x_vals);          

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
