#include "GeneticAlgorithm.h"
#include "SprayPathSequencer.h"
#include "HelpFunction.h"
#include <set>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime>

#define LOG_OUTPUT_DIR "E:\\vsproject\\SandSprayProject\\txtfile"

// ==================== 构造函数 ====================

GeneticAlgorithm::GeneticAlgorithm(
    SprayPathSequencer* seq,
    int startPath,
    bool startFromStart,
    int popSize,
    int maxGen,
    double crossRate,
    double mutRate,
    int elites)
    : sequencer(seq),
    startPathIndex(startPath),
    startFromPathStart(startFromStart),
    populationSize(popSize),
    maxGenerations(maxGen),
    crossoverRate(crossRate),
    mutationRate(mutRate),
    eliteCount(elites),
    bestFitness(0.0)  // 适应度为正值，从0开始
{
    // 初始化随机数生成器
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    rng.seed(seed + 12345);  // 添加偏移避免与 sequencer 的 rng 冲突

    // 初始化日志文件
    initializeLogFile();
}

// ==================== 析构函数 ====================

GeneticAlgorithm::~GeneticAlgorithm()
{
    if (logFile.is_open()) {
        logFile <<"==========================================================" << std::endl;
        logFile <<"遗传算法优化完成" << std::endl;
        logFile.close();
    }
}

// ==================== 日志文件初始化 ====================

void GeneticAlgorithm::initializeLogFile()
{
    // 生成带时间戳的文件名
    time_t now = time(0);
    struct tm timeinfo;
    localtime_s(&timeinfo, &now);

    std::stringstream ss;
    ss << "GeneticAlgorithm_"
        << (timeinfo.tm_year + 1900)
        << std::setfill('0') << std::setw(2) << (timeinfo.tm_mon + 1)
        << std::setfill('0') << std::setw(2) << timeinfo.tm_mday << "_"
        << std::setfill('0') << std::setw(2) << timeinfo.tm_hour
        << std::setfill('0') << std::setw(2) << timeinfo.tm_min
        << std::setfill('0') << std::setw(2) << timeinfo.tm_sec
        << ".txt";

    logFilePath = std::string(LOG_OUTPUT_DIR) + "\\" + ss.str();

    // 打开日志文件
    logFile.open(logFilePath, std::ios::out);
    if (logFile.is_open()) {
        logFile << "===================== 遗传算法优化日志 =====================" << std::endl;
        logFile << "算法参数配置:" << std::endl;
        logFile << "  种群大小: " << populationSize << std::endl;
        logFile << "  最大代数: " << maxGenerations << std::endl;
        logFile << "  交叉率: " << crossoverRate << std::endl;
        logFile << "  变异率: " << mutationRate << std::endl;
        logFile << "  精英个体数: " << eliteCount << std::endl;
        logFile << "  路径总数: " << sequencer->getPathCount() << std::endl;
        if (startPathIndex >= 0) {
            std::string startPoint = startFromPathStart ? "起点" : "终点";
            logFile << "  起点约束: 路径 " << startPathIndex << " (从" << startPoint << "开始)" << std::endl;
        }
        else {
            logFile << "  起点约束: 无约束" << std::endl;
        }
        logFile << "===========================================================" << std::endl;
        logFile << std::endl;
    }
}

// ==================== 适应度计算 ====================

double GeneticAlgorithm::calculateFitness(const Individual& individual)
{
    // 适应度 = 1/距离（距离越短，适应度越高）
    double totalDistance = sequencer->calculatePathLength(individual);
    if (totalDistance < 1e-6) return 1e6;  // 防止除零
    return 1.0 / totalDistance;
}

// ==================== 种群初始化 ====================

void GeneticAlgorithm::initializePopulation()
{
    population.clear();
    population.reserve(populationSize);

    int numPaths = sequencer->getPathCount();
    if (numPaths == 0) return;

    // 如果有起点约束，确定起点路径
    int actualStartPath = (startPathIndex >= 0) ? startPathIndex : 0;

    // 50% 贪心解（从不同起点）
    int greedyCount = populationSize / 2;
    for (int i = 0; i < greedyCount; i++) {
        if (startPathIndex >= 0) {
            // 有约束：所有贪心解从固定起点开始
            population.push_back(createGreedyIndividual(actualStartPath));
        }
        else {
            // 无约束：随机选择起点
            int randomStart = std::uniform_int_distribution<int>(0, numPaths - 1)(rng);
            population.push_back(createGreedyIndividual(randomStart));
        }
    }

    // 30% 随机解
    int randomCount = (populationSize * 3) / 10;
    for (int i = 0; i < randomCount; i++) {
        population.push_back(createRandomIndividual());
    }

    // 剩余 20% 变异贪心解
    int remainingCount = populationSize - greedyCount - randomCount;
    for (int i = 0; i < remainingCount; i++) {
        if (startPathIndex >= 0) {
            population.push_back(createMutatedGreedyIndividual(actualStartPath));
        }
        else {
            int randomStart = std::uniform_int_distribution<int>(0, numPaths - 1)(rng);
            population.push_back(createMutatedGreedyIndividual(randomStart));
        }
    }

    // 确保种群大小正确
    while (population.size() < populationSize) {
        population.push_back(createRandomIndividual());
    }

    // 评估初始种群
    evaluatePopulation();
    updateBestSolution();
}

GeneticAlgorithm::Individual GeneticAlgorithm::createGreedyIndividual(int startPath)
{
    // 调用 sequencer 的贪心算法
    return sequencer->greedyInitialSolution(startPath);
}

GeneticAlgorithm::Individual GeneticAlgorithm::createRandomIndividual()
{
    int numPaths = sequencer->getPathCount();
    Individual individual;
    individual.reserve(numPaths);

    // 创建路径索引列表
    std::vector<int> pathIndices;
    pathIndices.reserve(numPaths);

    // 如果有起点约束，第一个路径固定
    int fixedStart = (startPathIndex >= 0) ? startPathIndex : -1;

    if (fixedStart >= 0) {
        // 添加固定起点
        individual.push_back(std::make_pair(fixedStart, !startFromPathStart));

        // 其余路径
        for (int i = 0; i < numPaths; i++) {
            if (i != fixedStart) {
                pathIndices.push_back(i);
            }
        }
    }
    else {
        // 无约束，所有路径随机排列
        for (int i = 0; i < numPaths; i++) {
            pathIndices.push_back(i);
        }
    }

    // 随机打乱
    std::shuffle(pathIndices.begin(), pathIndices.end(), rng);

    // 添加到个体
    for (int idx : pathIndices) {
        bool randomDirection = std::uniform_int_distribution<int>(0, 1)(rng) == 1;
        individual.push_back(std::make_pair(idx, randomDirection));
    }

    return individual;
}

GeneticAlgorithm::Individual GeneticAlgorithm::createMutatedGreedyIndividual(int startPath)
{
    // 先创建贪心解
    Individual individual = createGreedyIndividual(startPath);

    // 应用轻微变异
    int numMutations = std::max(1, (int)individual.size() / 10);
    for (int i = 0; i < numMutations; i++) {
        mutate(individual);
    }

    return individual;
}

// ==================== 评估与更新 ====================

void GeneticAlgorithm::evaluatePopulation()
{
    fitness.clear();
    fitness.reserve(population.size());

    for (const auto& individual : population) {
        fitness.push_back(calculateFitness(individual));
    }
}

void GeneticAlgorithm::updateBestSolution()
{
    for (size_t i = 0; i < population.size(); i++) {
        if (fitness[i] > bestFitness) {
            bestFitness = fitness[i];
            bestIndividual = population[i];
        }
    }
}

// ==================== 选择算子 ====================

GeneticAlgorithm::Individual GeneticAlgorithm::tournamentSelection(int tournamentSize)
{
    std::uniform_int_distribution<int> dist(0, populationSize - 1);

    int bestIdx = dist(rng);
    double bestFit = fitness[bestIdx];

    for (int i = 1; i < tournamentSize; i++) {
        int idx = dist(rng);
        if (fitness[idx] > bestFit) {
            bestFit = fitness[idx];
            bestIdx = idx;
        }
    }

    return population[bestIdx];
}

// ==================== 交叉算子 ====================

std::pair<GeneticAlgorithm::Individual, GeneticAlgorithm::Individual>
GeneticAlgorithm::orderCrossover(const Individual& parent1, const Individual& parent2)
{
    int n = parent1.size();
    Individual child1(n), child2(n);

    // 如果有起点约束，第一个基因固定
    int startIdx = 0;
    if (startPathIndex >= 0) {
        child1[0] = parent1[0];  // 固定起点
        child2[0] = parent2[0];  // 固定起点
        startIdx = 1;
    }

    // 随机选择交叉区间 [pos1, pos2]
    std::uniform_int_distribution<int> dist(startIdx, n - 2);
    int pos1 = dist(rng);
    int pos2 = std::uniform_int_distribution<int>(pos1 + 1, n - 1)(rng);

    // 执行顺序交叉 (OX)
    // child1: 从 parent1 复制 [pos1, pos2]，其余按 parent2 顺序填充
    std::set<int> used1, used2;

    // 复制固定起点
    if (startPathIndex >= 0) {
        used1.insert(parent1[0].first);
        used2.insert(parent2[0].first);
    }

    // 复制交叉区间
    for (int i = pos1; i <= pos2; i++) {
        child1[i] = parent1[i];
        child2[i] = parent2[i];
        used1.insert(parent1[i].first);
        used2.insert(parent2[i].first);
    }

    // 填充 child1 的其余位置（按 parent2 顺序）
    int currentPos = startIdx;
    for (int i = startIdx; i < n; i++) {
        if (currentPos >= pos1 && currentPos <= pos2) {
            currentPos = pos2 + 1;
        }
        if (currentPos >= n) break;

        if (used1.find(parent2[i].first) == used1.end()) {
            child1[currentPos] = parent2[i];
            used1.insert(parent2[i].first);
            currentPos++;
        }
    }

    // 填充 child2 的其余位置（按 parent1 顺序）
    currentPos = startIdx;
    for (int i = startIdx; i < n; i++) {
        if (currentPos >= pos1 && currentPos <= pos2) {
            currentPos = pos2 + 1;
        }
        if (currentPos >= n) break;

        if (used2.find(parent1[i].first) == used2.end()) {
            child2[currentPos] = parent1[i];
            used2.insert(parent1[i].first);
            currentPos++;
        }
    }

    return std::make_pair(child1, child2);
}

// ==================== 变异算子 ====================

void GeneticAlgorithm::mutate(Individual& individual)
{
    double randVal = std::uniform_real_distribution<double>(0.0, 1.0)(rng);

    if (randVal < 0.7) {
        // 70% 交换变异
        swapMutation(individual);
    }
    else if (randVal < 0.9) {
        // 20% 反转变异
        inversionMutation(individual);
    }
    else {
        // 10% 翻转方向变异
        flipMutation(individual);
    }
}

void GeneticAlgorithm::swapMutation(Individual& individual)
{
    int n = individual.size();
    if (n <= 1) return;

    // 确定可变异范围
    int startIdx = (startPathIndex >= 0) ? 1 : 0;
    int range = n - startIdx;
    if (range <= 1) return;

    // 随机选择两个不同位置
    std::uniform_int_distribution<int> dist(startIdx, n - 1);
    int pos1 = dist(rng);
    int pos2 = dist(rng);
    while (pos2 == pos1 && range > 1) {
        pos2 = dist(rng);
    }

    // 交换
    std::swap(individual[pos1], individual[pos2]);
}

void GeneticAlgorithm::inversionMutation(Individual& individual)
{
    int n = individual.size();
    if (n <= 1) return;

    // 确定可变异范围
    int startIdx = (startPathIndex >= 0) ? 1 : 0;
    int range = n - startIdx;
    if (range <= 1) return;

    // 随机选择区间 [pos1, pos2]
    std::uniform_int_distribution<int> dist(startIdx, n - 2);
    int pos1 = dist(rng);
    int pos2 = std::uniform_int_distribution<int>(pos1 + 1, n - 1)(rng);

    // 反转区间
    std::reverse(individual.begin() + pos1, individual.begin() + pos2 + 1);
}

void GeneticAlgorithm::flipMutation(Individual& individual)
{
    int n = individual.size();
    if (n == 0) return;

    // 确定可变异范围
    int startIdx = (startPathIndex >= 0) ? 1 : 0;
    if (startIdx >= n) return;

    // 随机选择一个位置
    std::uniform_int_distribution<int> dist(startIdx, n - 1);
    int pos = dist(rng);

    // 翻转方向
    individual[pos].second = !individual[pos].second;
}

// ==================== 2-opt 局部优化 ====================

void GeneticAlgorithm::apply2Opt(Individual& individual)
{
    // 直接调用 sequencer 的 2-opt 方法
    sequencer->optimizeWith2Opt(individual);
}

// ==================== 主求解循环 ====================

GeneticAlgorithm::Individual GeneticAlgorithm::solve()
{
    // 初始化种群
    initializePopulation();

    if (logFile.is_open()) {
        logFile << "【进化过程】" << std::endl;
        logFile << "初始种群最优适应度: " << std::scientific << std::setprecision(6)
                << bestFitness << " (距离: " << std::fixed << std::setprecision(2)
                << (1.0 / bestFitness) << " mm)" << std::endl;
        logFile << std::endl;
    }

    // 进化循环
    for (int generation = 0; generation < maxGenerations; generation++) {
        std::vector<Individual> newPopulation;
        newPopulation.reserve(populationSize);

        // 精英保留
        std::vector<std::pair<double, int>> sortedFitness;
        for (size_t i = 0; i < population.size(); i++) {
            sortedFitness.push_back(std::make_pair(fitness[i], i));
        }
        std::sort(sortedFitness.rbegin(), sortedFitness.rend());

        for (int i = 0; i < eliteCount && i < populationSize; i++) {
            newPopulation.push_back(population[sortedFitness[i].second]);
        }

        // 生成新个体
        while (newPopulation.size() < populationSize) {
            // 选择父代
            Individual parent1 = tournamentSelection(3);
            Individual parent2 = tournamentSelection(3);

            // 交叉
            if (std::uniform_real_distribution<double>(0.0, 1.0)(rng) < crossoverRate) {
                auto children = orderCrossover(parent1, parent2);
                newPopulation.push_back(children.first);
                if (newPopulation.size() < populationSize) {
                    newPopulation.push_back(children.second);
                }
            }
            else {
                newPopulation.push_back(parent1);
                if (newPopulation.size() < populationSize) {
                    newPopulation.push_back(parent2);
                }
            }
        }

        // 变异
        for (size_t i = eliteCount; i < newPopulation.size(); i++) {
            if (std::uniform_real_distribution<double>(0.0, 1.0)(rng) < mutationRate) {
                mutate(newPopulation[i]);
            }
        }

        // 更新种群
        population = newPopulation;

        // 重新评估
        evaluatePopulation();

        // 每代对最优个体应用 2-opt
        int bestIdx = 0;
        double maxFit = fitness[0];
        for (size_t i = 1; i < fitness.size(); i++) {
            if (fitness[i] > maxFit) {
                maxFit = fitness[i];
                bestIdx = i;
            }
        }
        apply2Opt(population[bestIdx]);
        fitness[bestIdx] = calculateFitness(population[bestIdx]);

        // 更新全局最优
        updateBestSolution();

        // 每 10 代输出进度
        if ((generation + 1) % 10 == 0 || generation == 0 || generation == maxGenerations - 1) {
            if (logFile.is_open()) {
                logFile << "第 " << std::setw(3) << (generation + 1) << " 代: "
                        << "最优适应度 = " << std::scientific << std::setprecision(6) << bestFitness
                        << " (距离: " << std::fixed << std::setprecision(2)
                        << (1.0 / bestFitness) << " mm)" << std::endl;
            }
        }
    }

    // 输出最终结果
    if (logFile.is_open()) {
        logFile << std::endl;
        logFile << "===========================================================" << std::endl;
        logFile << "【优化结果】" << std::endl;
        logFile << "最优适应度: " << std::scientific << std::setprecision(6) << bestFitness << std::endl;
        logFile << "最优总距离: " << std::fixed << std::setprecision(2)
                << (1.0 / bestFitness) << " mm" << std::endl;
        logFile << "完成代数: " << maxGenerations << std::endl;
        logFile << "===========================================================" << std::endl;
    }

    return bestIndividual;
}
