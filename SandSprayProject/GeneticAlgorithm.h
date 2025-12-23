#ifndef GENETICALGORITHM_H_INCLUDED
#define GENETICALGORITHM_H_INCLUDED

#include <vector>
#include <random>
#include <algorithm>
#include <limits>
#include <fstream>
#include <string>

// 前向声明
class SprayPathSequencer;

// 遗传算法优化器
class GeneticAlgorithm {
private:
    // ==================== 核心参数 ====================
    SprayPathSequencer* sequencer;  // 序列器引用

    int populationSize;             // 种群大小
    int maxGenerations;             // 最大代数
    double crossoverRate;           // 交叉概率
    double mutationRate;            // 变异概率
    int eliteCount;                 // 精英个体数量

    // 起点约束
    int startPathIndex;             // 固定起点路径索引 (-1表示无约束)
    bool startFromPathStart;        // 固定起点端点

    // ==================== 种群数据 ====================
    using Individual = std::vector<std::pair<int, bool>>;  // 个体 = 路径序列
    std::vector<Individual> population;                     // 当前种群
    std::vector<double> fitness;                            // 适应度（负距离，越大越好）

    // 最优解跟踪
    Individual bestIndividual;
    double bestFitness;

    // 随机数生成器
    std::mt19937 rng;

    // 日志文件
    std::ofstream logFile;
    std::string logFilePath;

    // ==================== 私有方法 ====================

    // 初始化种群
    void initializePopulation();

    // 生成单个初始个体
    Individual createGreedyIndividual(int startPath);
    Individual createRandomIndividual();
    Individual createMutatedGreedyIndividual(int startPath);

    // 计算适应度 (负距离)
    double calculateFitness(const Individual& individual);

    // 选择算子 - 锦标赛选择
    Individual tournamentSelection(int tournamentSize = 3);

    // 交叉算子 - 顺序交叉 (OX)
    std::pair<Individual, Individual> orderCrossover(
        const Individual& parent1,
        const Individual& parent2);

    // 变异算子
    void mutate(Individual& individual);
    void swapMutation(Individual& individual);      // 交换变异
    void inversionMutation(Individual& individual); // 反转变异
    void flipMutation(Individual& individual);      // 翻转方向变异

    // 更新种群适应度
    void evaluatePopulation();

    // 更新最优解
    void updateBestSolution();

    // 应用 2-opt 局部优化到个体
    void apply2Opt(Individual& individual);

    // 初始化日志文件
    void initializeLogFile();

public:
    // ==================== 构造与析构 ====================
    GeneticAlgorithm(
        SprayPathSequencer* seq,
        int startPath = -1,
        bool startFromStart = true,
        int popSize = 60,
        int maxGen = 150,
        double crossRate = 0.85,
        double mutRate = 0.15,
        int elites = 3);

    ~GeneticAlgorithm();

    // ==================== 主求解接口 ====================

    // 执行遗传算法优化
    Individual solve();

    // 获取最优适应度
    double getBestFitness() const { return bestFitness; }
};

#endif // GENETICALGORITHM_H_INCLUDED
