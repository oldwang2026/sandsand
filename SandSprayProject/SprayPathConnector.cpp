#include "SprayPathConnector.h"
#include "HelpFunction.h" // 假设包含 ask_curve_point, CreateTempLine 等
#include <uf_modl.h>
#include <uf_curve.h>
#include <uf_obj.h>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <limits>

#define LOG_OUTPUT_DIR "E:\\vsproject\\SandSprayProject\\txtfile"

// ==================== 构造与析构 ====================

SprayPathConnector::SprayPathConnector()
    : startPathIndex(-1),
    startEntryIsStart(true),
    bestDistance(std::numeric_limits<double>::max())
{
    // 初始化随机数种子
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    rng.seed(seed);
}

SprayPathConnector::~SprayPathConnector()
{
    if (logFile.is_open()) {
        logFile.close();
    }
}

// ==================== 基础数据管理接口 (复用原 Sequencer 逻辑) ====================

void SprayPathConnector::clearPaths()
{
    paths.clear();
    // 清空缓存
    distCache.clear();
    finalSequence.clear();
    bestDistance = std::numeric_limits<double>::max();
    startPathIndex = -1;
    startEntryIsStart = true;
}

int SprayPathConnector::getPathCount() const
{
    return static_cast<int>(paths.size());
}

// 复用 SprayPathSequencer::addPathsFromUI 的逻辑
void SprayPathConnector::addPathsFromUI(const std::vector<NXOpen::TaggedObject*>& selectedCurves)
{
    int startId = static_cast<int>(paths.size());
    paths.reserve(paths.size() + selectedCurves.size());

    for (size_t i = 0; i < selectedCurves.size(); i++)
    {
        NXOpen::TaggedObject* obj = selectedCurves[i];
        tag_t curveTag = obj->Tag();

        // 获取曲线端点 (使用 HelpFunction 或 UF 函数)
        double sp[3], ep[3];
        // 假设 ask_curve_point 是 HelpFunction.h 中的封装，或者直接调用 UF
        // 这里沿用原来的 ask_curve_point 写法，如果它是自定义函数
        ask_curve_point(curveTag, 0.0, sp);
        ask_curve_point(curveTag, 1.0, ep);

        double curveLength = 0.0;
        UF_CURVE_ask_arc_length(curveTag, 0.0, 1.0, UF_MODL_UNITS_PART, &curveLength);

        // 读取属性 (完全复用原逻辑)
        int regionId = -1;
        FaceRegionType regionType = FaceRegionType::UNSORTED;

        try {
            NXOpen::NXObject* nxObj = dynamic_cast<NXOpen::NXObject*>(obj);
            if (nxObj != nullptr) {
                // ... (保留原本繁琐的属性读取 try-catch 块，确保稳健性) ...
                // 为节省篇幅，这里简写，实际代码请完全复制原 cpp 中的 try-catch 逻辑
                try {
                    int isSprayPath = nxObj->GetIntegerAttribute("IsSprayPath");
                    if (isSprayPath == 1) {
                        try { regionId = nxObj->GetIntegerAttribute("RegionTag"); }
                        catch (...) {}
                        // ... 读取 RegionType ...
                    }
                }
                catch (...) {}
            }
        }
        catch (...) {}

        paths.emplace_back(
            NXOpen::Point3d(sp[0], sp[1], sp[2]),
            NXOpen::Point3d(ep[0], ep[1], ep[2]),
            startId + static_cast<int>(i),
            curveTag,
            regionId,
            regionType,
            curveLength
        );
    }
}

// ==================== 起点约束接口 ====================

void SprayPathConnector::setStartByPickedPoint(const NXOpen::Point3d& pickedPoint)
{
    if (paths.empty()) {
        startPathIndex = -1;
        return;
    }

    double minDistance = std::numeric_limits<double>::max();
    int bestPathIndex = -1;
    bool bestIsStart = true; // true: 靠近start点

    for (size_t i = 0; i < paths.size(); i++)
    {
        double distToStart = PointDistance(paths[i].start, pickedPoint);
        if (distToStart < minDistance) {
            minDistance = distToStart;
            bestPathIndex = static_cast<int>(i);
            bestIsStart = true;
        }

        double distToEnd = PointDistance(paths[i].end, pickedPoint);
        if (distToEnd < minDistance) {
            minDistance = distToEnd;
            bestPathIndex = static_cast<int>(i);
            bestIsStart = false;
        }
    }

    startPathIndex = bestPathIndex;

    // 关键逻辑转换：
    // 如果用户选的点靠近 Start，意味着我们要从 Start 开始喷
    // 所以 入口是 Start (entryIsStart = true)
    // 如果用户选的点靠近 End，意味着我们要从 End 开始喷 (反向)
    // 所以 入口是 End (entryIsStart = false)
    startEntryIsStart = bestIsStart;
}

void SprayPathConnector::setStartByPickedCurve(NXOpen::TaggedObject* pickedCurve)
{
    if (paths.empty() || pickedCurve == nullptr) {
        startPathIndex = -1;
        return;
    }
    tag_t pickedTag = pickedCurve->Tag();
    for (size_t i = 0; i < paths.size(); i++) {
        if (paths[i].tag == pickedTag) {
            startPathIndex = static_cast<int>(i);
            startEntryIsStart = true; // 默认正向
            return;
        }
    }
    startPathIndex = -1;
}

void SprayPathConnector::clearStartConstraint()
{
    startPathIndex = -1;
    startEntryIsStart = true;
}

// ==================== 结果获取 ====================

std::vector<std::pair<int, bool>> SprayPathConnector::getFinalSequence() const {
    return finalSequence;
}

std::vector<NXOpen::TaggedObject*> SprayPathConnector::getOptimizedCurves() const {
    std::vector<NXOpen::TaggedObject*> optimizedCurves;
    optimizedCurves.reserve(finalSequence.size());
    for (const auto& item : finalSequence) {
        int idx = item.first;
        if (idx >= 0 && idx < paths.size()) {
            NXOpen::TaggedObject* obj = NXOpen::NXObjectManager::Get(paths[idx].tag);
            if (obj) optimizedCurves.push_back(obj);
        }
    }
    return optimizedCurves;
}

double SprayPathConnector::getBestDistance() const {
    return bestDistance;
}

// ==================== 辅助几何函数 ====================

double SprayPathConnector::PointDistance(const NXOpen::Point3d& p1, const NXOpen::Point3d& p2) {
    return std::sqrt(std::pow(p1.X - p2.X, 2) +
        std::pow(p1.Y - p2.Y, 2) +
        std::pow(p1.Z - p2.Z, 2));
}

// 静态辅助函数实现
NXOpen::Point3d SprayPathConnector::EntryPoint(const SprayPath& p, bool entryIsStart) {
    return entryIsStart ? p.start : p.end;
}

NXOpen::Point3d SprayPathConnector::ExitPoint(const SprayPath& p, bool entryIsStart) {
    // 如果是从 start 进，则是正向，从 end 出
    // 如果是从 end 进，则是反向，从 start 出
    return entryIsStart ? p.end : p.start;
}

// ==================== 核心算法基础设施 (DP & Cache) ====================

// 必须包含 limits 头文件
#include <limits> 

void SprayPathConnector::buildDistanceCache()
{
    int n = static_cast<int>(paths.size());

    // 初始化 4D 数组
    distCache.assign(n, std::vector<std::vector<std::vector<double>>>(
        n, std::vector<std::vector<double>>(
            2, std::vector<double>(2, 0.0)
        )));

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i == j) continue;

            for (int entryA = 0; entryA <= 1; entryA++) {
                bool isStartA = (entryA == 1);
                NXOpen::Point3d exitPointA = ExitPoint(paths[i], isStartA);

                for (int entryB = 0; entryB <= 1; entryB++) {
                    bool isStartB = (entryB == 1);
                    NXOpen::Point3d entryPointB = EntryPoint(paths[j], isStartB);

                    distCache[i][j][entryA][entryB] = PointDistance(exitPointA, entryPointB);
                }
            }
        }
    }
}

double SprayPathConnector::transitionCost(int aIdx, bool aEntryIsStart, int bIdx, bool bEntryIsStart) const
{
    return distCache[aIdx][bIdx][aEntryIsStart ? 1 : 0][bEntryIsStart ? 1 : 0];
}

std::pair<double, std::vector<bool>> SprayPathConnector::evaluateSequenceWithDP(const std::vector<int>& order) const
{
    int n = static_cast<int>(order.size());
    if (n == 0) return { 0.0, {} };

    // 使用 infinity 代替 1e15，更加严谨 (修改点 B2)
    double INF = std::numeric_limits<double>::infinity();

    // dp[i][0]: entryIsStart=false (End进)
    // dp[i][1]: entryIsStart=true  (Start进)
    std::vector<std::vector<double>> dp(n, std::vector<double>(2, INF));
    std::vector<std::vector<int>> parent(n, std::vector<int>(2, -1));

    // --- 1. 初始化第一个节点 ---
    int firstIdx = order[0];

    // 强制检查起点约束 (修改点 B1: 双重保险)
    if (startPathIndex != -1) {
        // 如果序列第一个不是指定起点，说明序列非法，返回无穷大
        if (firstIdx != startPathIndex) {
            return { INF, {} };
        }

        // 只有符合约束的方向初始化为 0，另一个方向为 INF
        if (startEntryIsStart) {
            dp[0][1] = 0.0; // valid
            dp[0][0] = INF; // invalid
        }
        else {
            dp[0][0] = 0.0; // valid
            dp[0][1] = INF; // invalid
        }
    }
    else {
        // 无约束：两个方向均可，代价为0
        dp[0][0] = 0.0;
        dp[0][1] = 0.0;
    }

    // --- 2. DP 递推 ---
    for (int i = 1; i < n; i++) {
        int prevPath = order[i - 1];
        int currPath = order[i];

        for (int currState = 0; currState <= 1; currState++) { // 当前状态
            bool currEntryIsStart = (currState == 1);

            for (int prevState = 0; prevState <= 1; prevState++) { // 前驱状态
                // 剪枝：如果前驱状态不可达，跳过
                if (dp[i - 1][prevState] == INF) continue;

                bool prevEntryIsStart = (prevState == 1);
                double cost = dp[i - 1][prevState] +
                    transitionCost(prevPath, prevEntryIsStart, currPath, currEntryIsStart);

                if (cost < dp[i][currState]) {
                    dp[i][currState] = cost;
                    parent[i][currState] = prevState;
                }
            }
        }
    }

    // --- 3. 回溯找最优解 ---
    double minTotalDist = std::min(dp[n - 1][0], dp[n - 1][1]);

    // 如果无解（例如路径断开或约束冲突），返回空
    if (minTotalDist == INF) return { INF, {} };

    std::vector<bool> optimalFlags(n);
    int currState = (dp[n - 1][1] < dp[n - 1][0]) ? 1 : 0;

    for (int i = n - 1; i >= 0; i--) {
        optimalFlags[i] = (currState == 1);
        currState = parent[i][currState];
    }

    return { minTotalDist, optimalFlags };
}

// ==================== SA 优化算法实现 ====================

void SprayPathConnector::solve()
{
    if (paths.empty()) return;

    // 1. 构建距离缓存
    buildDistanceCache();

    // 2. 生成初始解
    std::vector<int> currentOrder = generateInitialSequence();

    // 3. 执行模拟退火
    std::vector<int> bestOrder = simulatedAnnealing(currentOrder);

    // 4. 计算最终结果
    auto result = evaluateSequenceWithDP(bestOrder);

    // 5. 提交结果
    commitResult(bestOrder, result.second, result.first);
}

std::vector<int> SprayPathConnector::generateInitialSequence()
{
    int n = static_cast<int>(paths.size());
    std::vector<int> bestOrder;
    double minTotalDist = std::numeric_limits<double>::max();

    std::vector<int> startCandidates;
    if (startPathIndex != -1) {
        startCandidates.push_back(startPathIndex);
    }
    else {
        for (int i = 0; i < n; i++) startCandidates.push_back(i);
    }

    // 多起点贪心
    for (int startNode : startCandidates) {
        std::vector<int> currentOrder;
        std::vector<bool> visited(n, false);
        currentOrder.reserve(n);

        currentOrder.push_back(startNode);
        visited[startNode] = true;

        NXOpen::Point3d currPos;

        // (修改点 E: 优化无约束时的初始方向选择)
        if (startPathIndex != -1 && startNode == startPathIndex) {
            // 有约束：严格遵守
            currPos = ExitPoint(paths[startNode], startEntryIsStart);
        }
        else {
            // 无约束：为了贪心质量，我们暂时假设"更优"的出口是离最近邻更近的那个端点
            // 这里做一个简单的预判：找到最近的一个未访问点，看它离 Start 近还是 End 近
            double minToStart = std::numeric_limits<double>::max();
            double minToEnd = std::numeric_limits<double>::max();

            for (int i = 0; i < n; ++i) {
                if (i == startNode) continue;
                // 简单估算：只看中心点或端点距离
                double d1 = PointDistance(paths[startNode].start, paths[i].start);
                double d2 = PointDistance(paths[startNode].start, paths[i].end);
                double d3 = PointDistance(paths[startNode].end, paths[i].start);
                double d4 = PointDistance(paths[startNode].end, paths[i].end);

                minToStart = std::min(minToStart, std::min(d1, d2));
                minToEnd = std::min(minToEnd, std::min(d3, d4));
            }

            // 如果 Start 离别人更近，那我们最好从 End 进 Start 出 (Entry=End)
            // 如果 End 离别人更近，那我们最好从 Start 进 End 出 (Entry=Start)
            if (minToStart < minToEnd) {
                currPos = paths[startNode].start; // 从Start出
            }
            else {
                currPos = paths[startNode].end;   // 从End出
            }
        }

        // 贪心主体
        for (int i = 1; i < n; i++) {
            int bestNext = -1;
            double minDist = std::numeric_limits<double>::max();
            NXOpen::Point3d nextExitPos;

            for (int next = 0; next < n; next++) {
                if (!visited[next]) {
                    double dStart = PointDistance(currPos, paths[next].start);
                    double dEnd = PointDistance(currPos, paths[next].end);

                    if (dStart < minDist) {
                        minDist = dStart;
                        bestNext = next;
                        nextExitPos = paths[next].end; // 进start -> 出end
                    }
                    if (dEnd < minDist) {
                        minDist = dEnd;
                        bestNext = next;
                        nextExitPos = paths[next].start; // 进end -> 出start
                    }
                }
            }

            if (bestNext != -1) {
                visited[bestNext] = true;
                currentOrder.push_back(bestNext);
                currPos = nextExitPos;
            }
        }

        double cost = evaluateSequenceWithDP(currentOrder).first;
        if (cost < minTotalDist) {
            minTotalDist = cost;
            bestOrder = currentOrder;
        }
    }

    return bestOrder;
}

std::vector<int> SprayPathConnector::simulatedAnnealing(const std::vector<int>& initOrder)
{
    std::vector<int> currentOrder = initOrder;
    std::vector<int> bestOrder = initOrder;

    double currentCost = evaluateSequenceWithDP(currentOrder).first;
    double bestCost = currentCost;

    // SA 参数 (修改点 D: 调整参数以保证迭代次数)
    double T = currentCost * 0.1; // 稍微提高初始温度比例
    if (T < 1.0) T = 100.0;

    double alpha = 0.995; // 减缓降温速度，增加搜索深度
    int maxIter = 6000;

    // 根据路径数量动态调整
    if (paths.size() < 20) maxIter = 3000;
    if (paths.size() > 50) maxIter = 10000;

    std::uniform_real_distribution<double> distReal(0.0, 1.0);
    std::uniform_int_distribution<int> distOp(0, 2);

    // (修改点 D: 移除 T > T_min 的强制退出，确保跑够 maxIter，或者 T_min 设得极小)
    for (int iter = 0; iter < maxIter; iter++) {
        std::vector<int> nextOrder = currentOrder;

        int op = distOp(rng);
        if (op == 0) applySwap(nextOrder);
        else if (op == 1) applyRelocate(nextOrder);
        else apply2Opt(nextOrder);

        double nextCost = evaluateSequenceWithDP(nextOrder).first;
        double delta = nextCost - currentCost;

        if (delta < 0 || std::exp(-delta / T) > distReal(rng)) {
            currentOrder = nextOrder;
            currentCost = nextCost;

            if (currentCost < bestCost) {
                bestCost = currentCost;
                bestOrder = currentOrder;
            }
        }

        T *= alpha;
        // 如果温度过低，可以重置一下（Reheating），或者就让它保持低温进行纯爬山
        if (T < 1e-4) T = 1e-4;
    }

    // (修改点 G: 删除无效的 logFile 写入)

    return bestOrder;
}

// ==================== 扰动算子 ====================

int getPerturbStartIdx(int startPathIndex) {
    return (startPathIndex != -1) ? 1 : 0;
}

void SprayPathConnector::applySwap(std::vector<int>& order) {
    int n = static_cast<int>(order.size());
    int startIdx = getPerturbStartIdx(startPathIndex);
    if (n - startIdx < 2) return;

    std::uniform_int_distribution<int> dist(startIdx, n - 1);
    int i = dist(rng);
    int j = dist(rng);
    while (i == j) j = dist(rng);

    std::swap(order[i], order[j]);
}

void SprayPathConnector::applyRelocate(std::vector<int>& order) {
    int n = static_cast<int>(order.size());
    int startIdx = getPerturbStartIdx(startPathIndex);
    if (n - startIdx < 2) return;

    std::uniform_int_distribution<int> dist(startIdx, n - 1);
    int src = dist(rng);
    int dst = dist(rng);
    while (src == dst) dst = dist(rng);

    // (修改点 C1: 修复 insert 后索引偏移问题)
    int element = order[src];
    order.erase(order.begin() + src);

    // 如果 dst 在 src 后面，由于 erase 导致索引左移，dst 需要减 1
    if (dst > src) {
        dst--;
    }

    order.insert(order.begin() + dst, element);
}

void SprayPathConnector::apply2Opt(std::vector<int>& order) {
    int n = static_cast<int>(order.size());
    int startIdx = getPerturbStartIdx(startPathIndex);
    if (n - startIdx < 2) return;

    std::uniform_int_distribution<int> dist(startIdx, n - 1);
    int i = dist(rng);
    int j = dist(rng);

    if (i > j) std::swap(i, j);
    if (i == j) {
        if (j < n - 1) j++;
        else if (i > startIdx) i--;
        else return;
    }

    std::reverse(order.begin() + i, order.begin() + j + 1);
}

void SprayPathConnector::commitResult(const std::vector<int>& order,
    const std::vector<bool>& entryFlags,
    double totalAirMove)
{
    bestDistance = totalAirMove;
    finalSequence.clear();
    finalSequence.reserve(order.size());

    for (size_t i = 0; i < order.size(); i++) {
        finalSequence.push_back({ order[i], entryFlags[i] });
    }
}

// ==================== 可视化与日志 (辅助) ====================

void SprayPathConnector::printResult() {


    // 简单实现示例:
    if (logFile.is_open()) return; // 避免重复打开，或者在 solve 外部控制

    // 生成带时间戳的文件名 (代码同原 Sequencer)
    time_t now = time(0);
    struct tm timeinfo;
    localtime_s(&timeinfo, &now);
    std::stringstream ss;
    ss << LOG_OUTPUT_DIR << "\\SprayPath_SA_"
        << (timeinfo.tm_year + 1900) << std::setfill('0') << std::setw(2) << (timeinfo.tm_mon + 1)
        << std::setfill('0') << std::setw(2) << timeinfo.tm_mday << "_"
        << std::setfill('0') << std::setw(2) << timeinfo.tm_hour
        << std::setfill('0') << std::setw(2) << timeinfo.tm_min
        << std::setfill('0') << std::setw(2) << timeinfo.tm_sec << ".txt";

    logFile.open(ss.str(), std::ios::out);
    if (logFile.is_open()) {
        logFile << "===== 喷砂路径优化结果 (SA + DP) =====" << std::endl;
        logFile << "路径总数: " << paths.size() << std::endl;
        logFile << "最优空移距离: " << bestDistance << " mm" << std::endl;
        logFile << "----------------------------------------" << std::endl;
        for (size_t i = 0; i < finalSequence.size(); i++) {
            int idx = finalSequence[i].first;
            bool isStart = finalSequence[i].second;
            logFile << "Step " << std::setw(2) << i + 1 << ": Path " << std::setw(3) << idx
                << " | Dir: " << (isStart ? "Start->End" : "End->Start")
                << std::endl;
        }
        logFile.close();
    }
}

void SprayPathConnector::createConnectionLines() {
    // 遍历 finalSequence，使用 HelpFunction 中的 CreateTempLine 创建连接线
    // 逻辑与原 Sequencer 完全一致
    if (finalSequence.size() < 2) return;

    for (size_t i = 0; i < finalSequence.size() - 1; i++) {
        int idx1 = finalSequence[i].first;
        bool entry1 = finalSequence[i].second;

        int idx2 = finalSequence[i + 1].first;
        bool entry2 = finalSequence[i + 1].second;

        // Path1 的出口
        NXOpen::Point3d p1 = ExitPoint(paths[idx1], entry1);
        // Path2 的入口
        NXOpen::Point3d p2 = EntryPoint(paths[idx2], entry2);

        double sp[3] = { p1.X, p1.Y, p1.Z };
        double ep[3] = { p2.X, p2.Y, p2.Z };

        // 调用 HelpFunction (假设颜色 ID 186)
        CreateTempLine(sp, ep, 186);
    }
}