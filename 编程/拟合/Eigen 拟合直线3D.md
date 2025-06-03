```cpp
#include <Eigen/Dense>
#include <vector>

void fitLine(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& origin, Eigen::Vector3d& direction) {
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& p : points) mean += p;
    mean /= points.size();

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : points) cov += (p - mean) * (p - mean).transpose();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    direction = solver.eigenvectors().col(2).normalized(); // 最大特征值对应的特征向量
    origin = mean;
}

int main() {
    std::vector<Eigen::Vector3d> points;
    // 添加 double 类型的点
    for (int i = 0; i < 100; ++i) {
        points.emplace_back(0.1 * i, 0.2 * i, 0.3 * i);
    }

    Eigen::Vector3d origin, direction;
    fitLine(points, origin, direction);
    std::cout << "Origin: " << origin.transpose() << "\nDirection: " << direction.transpose() << std::endl;
    return 0;
}
```