#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

class KalmanFilter2D {
private:
    double x1, x2;  // 价格, 变化率
    double P11, P12, P21, P22;
    double Q11, Q22, R;
public:
    KalmanFilter2D(double Qp, double Qr, double Rm, double init_price, double init_P)
        : x1(init_price), x2(0), P11(init_P), P12(0), P21(0), P22(init_P),
          Q11(Qp), Q22(Qr), R(Rm) {}
    
    void predict() {
        x1 = x1 + x2;
        // x2 = x2;
        P11 = P11 + 2*P12 + P22 + Q11;
        P12 = P12 + P22;
        P21 = P21 + P22;
        P22 = P22 + Q22;
    }
    
    void update(double z) {
        double K1 = P11 / (P11 + R);
        double K2 = P21 / (P11 + R);
        double res = z - x1;
        x1 = x1 + K1 * res;
        x2 = x2 + K2 * res;
        P11 = P11 - K1 * P11;
        P12 = P12 - K1 * P12;
        P21 = P21 - K2 * P11;
        P22 = P22 - K2 * P12;
    }
    
    double filter(double z) {
        predict();
        update(z);
        return x1;
    }
    
    double predictFuture(int steps) {
        return x1 + x2 * steps;
    }
    
    double getRate() { return x2; }
};

int main() {
    std::ifstream file("kalman_filter/stock_prices.csv");
    if (!file.is_open()) {
        std::cout << "无法打开文件\n";
        return 1;
    }
    
    std::vector<double> prices;
    std::string line;
    bool first = true;
    while (std::getline(file, line)) {
        if (first) { first = false; continue; }  // 跳过表头
        if (line.empty()) continue;
        size_t comma = line.find(',');
        double price = std::stod(line.substr(comma + 1));
        prices.push_back(price);
    }
    file.close();
    
    KalmanFilter2D kf(0.01, 0.001, 1.0, prices[0], 10.0);
    std::vector<double> filtered;
    for (double p : prices) {
        filtered.push_back(kf.filter(p));
    }
    
    int predict_days = 10;
    std::cout << "\n未来 " << predict_days << " 天预测:\n";
    for (int i = 1; i <= predict_days; i++) {
        std::cout << "第 " << i << " 天: " << kf.predictFuture(i) << std::endl;
    }
    
    // 保存CSV
    std::ofstream out("zuoye2_result.csv");
    out << "day,original,filtered,prediction\n";
    for (size_t i = 0; i < prices.size(); i++) {
        out << i+1 << "," << prices[i] << "," << filtered[i] << ",\n";
    }
    for (int i = 1; i <= predict_days; i++) {
        out << prices.size() + i << ",,," << kf.predictFuture(i) << "\n";
    }
    out.close();
    
    std::cout << "\n结果已保存,运行 python zuoye2.py 画图" << std::endl;
    
    return 0;
}