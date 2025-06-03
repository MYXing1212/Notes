
Header Only
析构时自动打印执行时间

```cpp
#pragma once
#include <chrono>
#include <iostream>
#include <string>

class Timer {
public:
    // 构造函数：记录开始时间，可选指定标签
    explicit Timer(std::string name = "Block")
        : name_(std::move(name)),
        start_(std::chrono::high_resolution_clock::now()) {
    }

    // 析构函数：自动计算并打印耗时
    ~Timer() {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_);
        std::cout << name_ << " took: " << duration.count() << " μs\n";
    }

    // 禁止拷贝和移动
    Timer(const Timer&) = delete;
    Timer& operator=(const Timer&) = delete;

private:
    std::string name_;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

```