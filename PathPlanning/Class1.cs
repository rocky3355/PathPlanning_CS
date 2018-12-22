/*
List<double> ptsx = new List<double>();
List<double> ptsy = new List<double>();

double ref_vel = 49.5;
double ref_x = msg.x;
double ref_y = msg.y;
double ref_yaw = deg2rad(msg.yaw);
int prev_size = msg.previous_path_x.Length;

if (prev_size < 2) {
    // TODO: deg2rad?
    double prev_car_x = msg.x - Math.Cos(msg.yaw);
    double prev_car_y = msg.x - Math.Sin(msg.yaw);
    ptsx.Add(prev_car_x);
    ptsx.Add(msg.x);
    ptsy.Add(prev_car_y);
    ptsy.Add(msg.y);
}
else {
    ref_x = msg.previous_path_x[prev_size - 1];
    ref_y = msg.previous_path_y[prev_size - 1];
    double ref_x_prev = msg.previous_path_x[prev_size - 2];
    double ref_y_prev = msg.previous_path_y[prev_size - 2];
    ref_yaw = Math.Atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    ptsx.Add(ref_x_prev);
    ptsx.Add(ref_x);
    ptsy.Add(ref_y_prev);
    ptsy.Add(ref_y);
}

int lane = 1;
double[] next_wp0 = getXY(msg.s + 30.0, 2 + 4 * lane);
double[] next_wp1 = getXY(msg.s + 60.0, 2 + 4 * lane);
double[] next_wp2 = getXY(msg.s + 90.0, 2 + 4 * lane);
ptsx.Add(next_wp0[0]);
ptsx.Add(next_wp1[0]);
ptsx.Add(next_wp2[0]);
ptsy.Add(next_wp0[1]);
ptsy.Add(next_wp1[1]);
ptsy.Add(next_wp2[1]);

for (int i = 0; i < ptsx.Count; i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    // TODO: Precalculate sin and cos
    ptsx[i] = shift_x * Math.Cos(0 - ref_yaw) - shift_y * Math.Sin(0 - ref_yaw);
    ptsx[i] = shift_x * Math.Sin(0 - ref_yaw) + shift_y * Math.Cos(0 - ref_yaw);
}

CubicSpline spline = new CubicSpline(ptsx.ToArray(), ptsy.ToArray());

List<double> next_x_vals = new List<double>();
List<double> next_y_vals = new List<double>();

for (int i = 0; i < prev_size; i++) {
    next_x_vals.Add(msg.previous_path_x[i]);
    next_y_vals.Add(msg.previous_path_y[i]);
}

double target_x = 30.0;
double target_y = spline.Eval(target_x);
double target_dist = Math.Sqrt(target_x * target_x + target_y * target_y);

double x_add_on = 0.0;

for (int i = 1; i <= 50 - prev_size; i++) {
    double N = target_dist / (0.02 * ref_vel / 2.24);
    double x_point = x_add_on + target_x / N;
    double y_point = spline.Eval(x_point);
    x_add_on = x_point;
    double x_ref = x_point;
    double y_ref = y_point;

    // TODO: Precalculate sin and cos (same value as above)
    x_point = x_ref * Math.Cos(ref_yaw) - y_ref * Math.Sin(ref_yaw);
    y_point = x_ref * Math.Sin(ref_yaw) + y_ref * Math.Cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.Add(x_point);
    next_y_vals.Add(y_point);
}
*/