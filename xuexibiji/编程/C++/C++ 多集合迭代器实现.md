
下面是一个完整的 C++ 实现，展示了如何为类 `A` 添加多种不同的迭代器来遍历其包含的两个 `B` 类集合：

```cpp
#include <iostream>
#include <vector>
#include <memory>

// 前置声明
class B;

class A {
private:
    std::vector<std::shared_ptr<B>> collection1;
    std::vector<std::shared_ptr<B>> collection2;

public:
    // 添加元素到集合1和集合2
    void addToCollection1(std::shared_ptr<B> b) { collection1.push_back(b); }
    void addToCollection2(std::shared_ptr<B> b) { collection2.push_back(b); }

    // 迭代器基类
    class Iterator {
    public:
        virtual ~Iterator() = default;
        virtual void first() = 0;
        virtual void next() = 0;
        virtual bool isDone() const = 0;
        virtual std::shared_ptr<B> currentItem() const = 0;
    };

    // 集合1的迭代器
    class Collection1Iterator : public Iterator {
        const std::vector<std::shared_ptr<B>>& collection;
        size_t current;
    public:
        explicit Collection1Iterator(const std::vector<std::shared_ptr<B>>& coll) 
            : collection(coll), current(0) {}

        void first() override { current = 0; }
        void next() override { if (!isDone()) ++current; }
        bool isDone() const override { return current >= collection.size(); }
        std::shared_ptr<B> currentItem() const override { 
            return isDone() ? nullptr : collection[current]; 
        }
    };

    // 集合2的迭代器
    class Collection2Iterator : public Iterator {
        const std::vector<std::shared_ptr<B>>& collection;
        size_t current;
    public:
        explicit Collection2Iterator(const std::vector<std::shared_ptr<B>>& coll) 
            : collection(coll), current(0) {}

        void first() override { current = 0; }
        void next() override { if (!isDone()) ++current; }
        bool isDone() const override { return current >= collection.size(); }
        std::shared_ptr<B> currentItem() const override { 
            return isDone() ? nullptr : collection[current]; 
        }
    };

    // 同时遍历两个集合的迭代器
    class CombinedIterator : public Iterator {
        const std::vector<std::shared_ptr<B>>& coll1;
        const std::vector<std::shared_ptr<B>>& coll2;
        size_t current;
        bool inFirstCollection;
    public:
        CombinedIterator(const std::vector<std::shared_ptr<B>>& c1, 
                        const std::vector<std::shared_ptr<B>>& c2)
            : coll1(c1), coll2(c2), current(0), inFirstCollection(true) {}

        void first() override { 
            current = 0; 
            inFirstCollection = true;
        }

        void next() override { 
            if (isDone()) return;
            
            if (inFirstCollection) {
                ++current;
                if (current >= coll1.size()) {
                    inFirstCollection = false;
                    current = 0;
                }
            } else {
                ++current;
            }
        }

        bool isDone() const override { 
            return !inFirstCollection && current >= coll2.size(); 
        }

        std::shared_ptr<B> currentItem() const override { 
            if (isDone()) return nullptr;
            return inFirstCollection ? coll1[current] : coll2[current]; 
        }
    };

    // 创建各种迭代器的工厂方法
    std::unique_ptr<Iterator> createCollection1Iterator() const {
        return std::make_unique<Collection1Iterator>(collection1);
    }

    std::unique_ptr<Iterator> createCollection2Iterator() const {
        return std::make_unique<Collection2Iterator>(collection2);
    }

    std::unique_ptr<Iterator> createCombinedIterator() const {
        return std::make_unique<CombinedIterator>(collection1, collection2);
    }
};

// 简单的B类实现
class B {
private:
    int id;
public:
    explicit B(int id) : id(id) {}
    int getId() const { return id; }
};

// 打印迭代器内容的辅助函数
void printIterator(A::Iterator& iterator) {
    for (iterator.first(); !iterator.isDone(); iterator.next()) {
        auto item = iterator.currentItem();
        if (item) {
            std::cout << item->getId() << " ";
        }
    }
    std::cout << std::endl;
}

int main() {
    A a;
    
    // 添加一些测试数据
    a.addToCollection1(std::make_shared<B>(1));
    a.addToCollection1(std::make_shared<B>(2));
    a.addToCollection1(std::make_shared<B>(3));
    
    a.addToCollection2(std::make_shared<B>(10));
    a.addToCollection2(std::make_shared<B>(20));
    a.addToCollection2(std::make_shared<B>(30));
    a.addToCollection2(std::make_shared<B>(40));

    // 测试各种迭代器
    std::cout << "Collection1: ";
    auto iter1 = a.createCollection1Iterator();
    printIterator(*iter1);

    std::cout << "Collection2: ";
    auto iter2 = a.createCollection2Iterator();
    printIterator(*iter2);

    std::cout << "Combined: ";
    auto combinedIter = a.createCombinedIterator();
    printIterator(*combinedIter);

    return 0;
}
```

## 代码说明

1. **类结构**:
   - `A` 类包含两个 `B` 类的集合 (`collection1` 和 `collection2`)
   - 使用 `shared_ptr` 管理 `B` 对象生命周期

2. **迭代器设计**:
   - 抽象基类 `Iterator` 定义了通用接口
   - `Collection1Iterator` 和 `Collection2Iterator` 分别遍历单个集合
   - `CombinedIterator` 按顺序遍历两个集合

3. **迭代器行为**:
   - `first()` 重置迭代器到起始位置
   - `next()` 移动到下一个元素
   - `isDone()` 检查是否遍历完成
   - `currentItem()` 获取当前元素

4. **工厂方法**:
   - `createCollection1Iterator()`
   - `createCollection2Iterator()`
   - `createCombinedIterator()`

5. **使用智能指针**:
   - 使用 `unique_ptr` 管理迭代器对象
   - 使用 `shared_ptr` 管理 `B` 实例

## 输出示例

运行程序将输出类似以下内容:
```
Collection1: 1 2 3 
Collection2: 10 20 30 40 
Combined: 1 2 3 10 20 30 40 
```

这种设计遵循了迭代器模式，提供了统一的接口来遍历不同的集合，同时保持了集合实现的封装性。