#include <iostream>
#include <fstream>
#include <vector>
#include <string>

class KalmanFilter2D {
private:
    double x1, x2;
    double P11, P12, P21, P22;
    double Q11, Q22, R;
public:
    KalmanFilter2D(double Qp, double Qr, double Rm, double init_price, double init_P)
        : x1(init_price), x2(0), P11(init_P), P12(0), P21(0), P22(init_P),
          Q11(Qp), Q22(Qr), R(Rm) {}
    
    void predict() {
        x1 = x1 + x2;
        P11 = P11 + 2*P12 + P22 + Q11;
        P12 = P12 + P22;
        P21 = P21 + P22;
        P22 = P22 + Q22;
    }
    
    void update(double z) {
        double denom = P11 + R;
        double K1 = P11 / denom;
        double K2 = P21 / denom;
        double res = z - x1;
        x1 = x1 + K1 * res;
        x2 = x2 + K2 * res;
        
        double new_P11 = P11 - K1 * P11;
        double new_P12 = P12 - K1 * P12;
        double new_P21 = P21 - K2 * P11;
        double new_P22 = P22 - K2 * P12;
        
        P11 = new_P11;
        P12 = new_P12;
        P21 = new_P21;
        P22 = new_P22;
    }
    
    double filter(double z) {
        predict();
        update(z);
        return x1;
    }
    
    double getRate() { return x2; }
    double getPrice() { return x1; }
};

int main() {
    std::ifstream file("kalman_filter/stock_prices.csv");
    if (!file.is_open()) {
        std::cout << "Cannot open file" << std::endl;
        return 1;
    }
    
    std::vector<double> prices;
    std::string line;
    bool first_line = true;
    while (std::getline(file, line)) {
        if (first_line) {
            first_line = false;
            continue;
        }
        if (line.empty()) continue;
        
        size_t comma = line.find(',');
        double price = std::stod(line.substr(comma + 1));
        prices.push_back(price);
    }
    file.close();
    
    KalmanFilter2D kf(0.001, 0.5, 10.0, prices[0], 10.0);
    
    std::vector<double> filtered;
    for (double p : prices) {
        filtered.push_back(kf.filter(p));
    }
    
    int predict_days = 10;
    std::vector<double> predictions;
    double future_price = filtered.back();
    double future_rate = kf.getRate();
    
    for (int i = 1; i <= predict_days; i++) {
        future_price = future_price + future_rate;
        predictions.push_back(future_price);
    }
    
    std::ofstream out("zuoye2_result.csv");
    out << "day,original,filtered,prediction" << std::endl;
    
    for (size_t i = 0; i < prices.size(); i++) {
        out << i+1 << "," << prices[i] << "," << filtered[i] << "," << std::endl;
    }
    
    
    out.close();
    
    std::cout << "Saved to zuoye2_result.csv" << std::endl;
    std::cout << "Run: python3 zuoye2.py" << std::endl;
    
    return 0;
}