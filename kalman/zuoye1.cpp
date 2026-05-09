#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>

class KalmanFilter1D {
private:
    double x, P, Q, R;
public:
    KalmanFilter1D(double Q, double R, double init_x, double init_P)
        : Q(Q), R(R), x(init_x), P(init_P) {}
    
    double filter(double z) {
        double x_pred = x;
        double P_pred = P + Q;
        double K = P_pred / (P_pred + R);
        x = x_pred + K * (z - x_pred);
        P = (1 - K) * P_pred;
        return x;
    }
};

void processFile(const std::string& inputPath, const std::string& outputPath, int fileNum, double Q, double R) {
    std::ifstream file(inputPath);
    if (!file.is_open()) {
        std::cout << "Cannot open file: " << inputPath << std::endl;
        return;
    }
    
    std::vector<double> times, values;
    double t, v;
    while (file >> t >> v) {
        times.push_back(t);
        values.push_back(v);
    }
    file.close();
    
    if (times.empty()) return;
    
    KalmanFilter1D kf(Q, R, values[0], 1.0);
    std::vector<double> filtered;
    for (double v : values) {
        filtered.push_back(kf.filter(v));
    }
    
    int n = times.size();
    double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
    for (int i = 0; i < n; i++) {
        sum_x += times[i];
        sum_y += filtered[i];
        sum_xx += times[i] * times[i];
        sum_xy += times[i] * filtered[i];
    }
    double a = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
    double b = (sum_y - a * sum_x) / n;
    
    std::cout << "File " << fileNum << " fitted: y = " << a << " * t + " << b << std::endl;
    
    std::ofstream out(outputPath);
    out << "time,original,filtered,fitted\n";
    for (int i = 0; i < n; i++) {
        out << times[i] << "," << values[i] << "," << filtered[i] << "," << a * times[i] + b << "\n";
    }
    out.close();
}

int main() {
    std::cout << "========== 作业 1 ==========\n\n";
    
    processFile("kalman_filter/homework_data_1.txt", "zuoye1_1.csv", 1, 0.005, 3.0);
    processFile("kalman_filter/homework_data_2.txt", "zuoye1_2.csv", 2, 0.01, 1.0);
    processFile("kalman_filter/homework_data_3.txt", "zuoye1_3.csv", 3, 1.0, 25.0);
    processFile("kalman_filter/homework_data_4.txt", "zuoye1_4.csv", 4, 0.001, 3.0);
    
    std::cout << "\nDone! Run python zuoye1.py to plot" << std::endl;
    
    return 0;
}