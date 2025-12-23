#include "SprayPathSequencer.h"
#include "GeneticAlgorithm.h"
#include "HelpFunction.h"
#include <iomanip>
#include <sstream>
#include <map>
#include <uf_obj.h>


#define LOG_OUTPUT_DIR "E:\\vsproject\\SandSprayProject\\txtfile"

SprayPath::SprayPath(const NXOpen::Point3d& s, const NXOpen::Point3d& e,
    int pathId, tag_t curveTag, int regId, FaceRegionType type, double len)
    : start(s), end(e), id(pathId), tag(curveTag), regionId(regId), regionType(type), length(len)
{
}

PathEdge::PathEdge(int p1, int p2, bool s1, bool s2, double dist)
    : path1(p1), path2(p2), isStart1(s1), isStart2(s2), distance(dist)
{
}

bool PathEdge::operator<(const PathEdge& other) const {
    return distance < other.distance;
}

SprayPathSequencer::SprayPathSequencer()
    : bestDistance(std::numeric_limits<double>::max()),
    initialDistance(0.0),
    startPathIndex(-1),
    startFromPathStart(true)
{
    // 初始化随机数生成器
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    rng.seed(seed);
}

SprayPathSequencer::~SprayPathSequencer()
{
    // 如果日志文件还开着，关闭它
    if (logFile.is_open()) {
        logFile.close();
    }
}

void SprayPathSequencer::clearPaths()
{
    paths.clear();
    edgeList.clear();
    usedPathStart.clear();
    usedPathEnd.clear();
    finalSequence.clear();

    bestDistance = std::numeric_limits<double>::max();
    startPathIndex = -1;
}

int SprayPathSequencer::getPathCount() const
{
    return static_cast<int>(paths.size());
}

void SprayPathSequencer::addPathsFromUI(const std::vector<NXOpen::TaggedObject*>& selectedCurves)
{
    int startId = static_cast<int>(paths.size());
    paths.reserve(paths.size() + selectedCurves.size());

    for (size_t i = 0; i < selectedCurves.size(); i++)
    {
        NXOpen::TaggedObject* obj = selectedCurves[i];
        tag_t curveTag = obj->Tag();

        // 获取曲线起点和终点
        double sp[3], ep[3];
        ask_curve_point(curveTag, 0.0, sp);
        ask_curve_point(curveTag, 1.0, ep);

        // 计算曲线长度
        double curveLength = 0.0;
        UF_CURVE_ask_arc_length(curveTag, 0.0, 1.0, UF_MODL_UNITS_PART, &curveLength);

        // 关键步骤：读取曲线属性
        int regionId = -1;
        FaceRegionType regionType = FaceRegionType::UNSORTED;  // 默认未分类

        try {
            NXOpen::NXObject* nxObj = dynamic_cast<NXOpen::NXObject*>(obj);

            if (nxObj != nullptr) {
                // 1. 检查是否为喷砂路径
                int isSprayPath = 0;
                try {
                    isSprayPath = nxObj->GetIntegerAttribute("IsSprayPath");
                }
                catch (...) {
                    // 如果不是喷砂路径，跳过后续属性读取
                }

                if (isSprayPath == 1) {
                    // 2. 获取区域Body Tag
                    try {
                        regionId = nxObj->GetIntegerAttribute("RegionTag");
                    }
                    catch (...) {
                        regionId = -1;
                    }

                    // 3. 获取区域类型（优先整数）
                    try {
                        int typeValue = nxObj->GetIntegerAttribute("RegionType");
                        regionType = static_cast<FaceRegionType>(typeValue);
                    }
                    catch (...) {
                        // 整数读取失败，尝试从字符串属性读取
                        try {
                            NXOpen::NXString typeName = nxObj->GetStringAttribute("RegionTypeName");
                            std::string typeStr = typeName.GetUTF8Text();

                            if (typeStr == "Horizontal") {
                                regionType = FaceRegionType::HORIZONTAL;
                            }
                            else if (typeStr == "Convex") {
                                regionType = FaceRegionType::CONVEX;
                            }
                            else if (typeStr == "Concave") {
                                regionType = FaceRegionType::CONCAVE;
                            }
                            else if (typeStr == "Sidewall") {
                                regionType = FaceRegionType::SIDEWALL;
                            }
                            else {
                                regionType = FaceRegionType::UNSORTED;
                            }
                        }
                        catch (...) {
                            regionType = FaceRegionType::UNSORTED;
                        }
                    }
                }
            }
        }
        catch (...) {
            // 异常捕获，使用默认值
            regionId = -1;
            regionType = FaceRegionType::UNSORTED;
        }

        // 创建SprayPath对象
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

    // 调整使用状态向量大小
    usedPathStart.resize(paths.size(), false);
    usedPathEnd.resize(paths.size(), false);
}

void SprayPathSequencer::clearStartConstraint()
{
    startPathIndex = -1;
    startFromPathStart = true;
}

void SprayPathSequencer::setStartByPickedPoint(const NXOpen::Point3d& pickedPoint)
{
    if (paths.empty()) {
        startPathIndex = -1;
        return;
    }

    double minDistance = std::numeric_limits<double>::max();
    int bestPathIndex = -1;
    bool bestIsStart = true;

    // 遍历所有路径的起点和终点，找到最近的端点
    for (size_t i = 0; i < paths.size(); i++)
    {
        // 计算到起点的距离
        double distToStart = PointDistance(paths[i].start, pickedPoint);

        if (distToStart < minDistance) {
            minDistance = distToStart;
            bestPathIndex = static_cast<int>(i);
            bestIsStart = true;
        }

        // 计算到终点的距离
        double distToEnd = PointDistance(paths[i].end, pickedPoint);

        if (distToEnd < minDistance) {
            minDistance = distToEnd;
            bestPathIndex = static_cast<int>(i);
            bestIsStart = false;
        }
    }

    // 如果找到了最近的端点，设置为起点
    startPathIndex = bestPathIndex;
    startFromPathStart = bestIsStart;
}

void SprayPathSequencer::setStartByPickedCurve(NXOpen::TaggedObject* pickedCurve)
{
    if (paths.empty() || pickedCurve == nullptr) {
        startPathIndex = -1;
        return;
    }

    tag_t pickedTag = pickedCurve->Tag();

    // 在paths中查找匹配的曲线
    for (size_t i = 0; i < paths.size(); i++)
    {
        if (paths[i].tag == pickedTag) {
            startPathIndex = static_cast<int>(i);
            startFromPathStart = true;  // 默认从起点开始
            return;
        }
    }

    // 没找到匹配的曲线
    startPathIndex = -1;
}

void SprayPathSequencer::computeAllEdges()
{
    int n = static_cast<int>(paths.size());
    edgeList.resize(n);

    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            // 计算四种可能的连接方式
            double d1 = PointDistance(paths[i].start, paths[j].start);
            double d2 = PointDistance(paths[i].start, paths[j].end);
            double d3 = PointDistance(paths[i].end, paths[j].start);
            double d4 = PointDistance(paths[i].end, paths[j].end);

            edgeList[i].emplace_back(i, j, true, true, d1);
            edgeList[i].emplace_back(i, j, true, false, d2);
            edgeList[i].emplace_back(i, j, false, true, d3);
            edgeList[i].emplace_back(i, j, false, false, d4);
        }

        // 对每条路径的边列表按距离排序
        std::sort(edgeList[i].begin(), edgeList[i].end());
    }
}

std::vector<std::pair<int, bool>> SprayPathSequencer::greedyInitialSolution(int startPath)
{
    // 重置所有使用标记
    std::fill(usedPathStart.begin(), usedPathStart.end(), false);
    std::fill(usedPathEnd.begin(), usedPathEnd.end(), false);

    std::vector<std::pair<int, bool>> sequence;

    sequence.push_back(std::make_pair(startPath, true));  // 从起点开始
    usedPathStart[startPath] = true;

    int remainingPaths = static_cast<int>(paths.size()) - 1;
    int currentPath = startPath;
    bool isCurrentStart = true;  // true = 起点, false = 终点

    while (remainingPaths > 0) {
        int bestNextPath = -1;
        bool bestNextIsStart = false;
        double bestDist = std::numeric_limits<double>::max();

        // 获取当前端点
        const NXOpen::Point3d& currentPoint = isCurrentStart ? paths[currentPath].end : paths[currentPath].start;

        // 遍历所有可能的下一条路径
        for (int nextPath = 0; nextPath < paths.size(); nextPath++)
        {
            if (nextPath == currentPath || (usedPathStart[nextPath] || usedPathEnd[nextPath]))
            {
                continue;  // 跳过已使用的路径
            }

            // 尝试连接到下一条路径的起点
            if (!usedPathStart[nextPath]) {
                double dist = PointDistance(currentPoint, paths[nextPath].start);
                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestNextPath = nextPath;
                    bestNextIsStart = true;
                }
            }

            // 尝试连接到下一条路径的终点
            if (!usedPathEnd[nextPath]) {
                double dist = PointDistance(currentPoint, paths[nextPath].end);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestNextPath = nextPath;
                    bestNextIsStart = false;
                }
            }
        }

        if (bestNextPath == -1) {
            break;  // 没有可连接的路径了
        }

        // 添加到路径序列
        sequence.push_back(std::make_pair(bestNextPath, bestNextIsStart));

        // 标记为使用
        if (bestNextIsStart) {
            usedPathStart[bestNextPath] = true;
        }
        else {
            usedPathEnd[bestNextPath] = true;
        }

        // 更新当前状态
        currentPath = bestNextPath;
        isCurrentStart = !bestNextIsStart;  // 如果连接到起点，下一次从终点出发，反之亦然

        remainingPaths--;
    }

    return sequence;
}

double SprayPathSequencer::calculatePathLength(const std::vector<std::pair<int, bool>>& sequence)
{
    if (sequence.size() <= 1) return 0.0;

    double totalLength = 0.0;
    for (size_t i = 0; i < sequence.size() - 1; i++) {
        int path1 = sequence[i].first;
        int path2 = sequence[i + 1].first;
        bool isStart1 = sequence[i].second;
        bool isStart2 = sequence[i + 1].second;

        const NXOpen::Point3d& p1 = isStart1 ? paths[path1].end : paths[path1].start;
        const NXOpen::Point3d& p2 = isStart2 ? paths[path2].start : paths[path2].end;

        totalLength += PointDistance(p1, p2);
    }

    return totalLength;
}

void SprayPathSequencer::optimizeWith2Opt(std::vector<std::pair<int, bool>>& sequence)
{
    bool improved = true;
    int iterations = 0;
    const int MAX_ITERATIONS = 1000;

    while (improved && iterations < MAX_ITERATIONS) {
        improved = false;
        iterations++;

        for (size_t i = 0; i < sequence.size() - 2; i++) {
            for (size_t j = i + 2; j < sequence.size() - 1; j++) {
                // 获取当前边
                int c1 = sequence[i].first;
                int c2 = sequence[i + 1].first;
                int c3 = sequence[j].first;
                int c4 = sequence[j + 1].first;

                bool s1 = sequence[i].second;
                bool s2 = sequence[i + 1].second;
                bool s3 = sequence[j].second;
                bool s4 = sequence[j + 1].second;

                const NXOpen::Point3d& p1 = s1 ? paths[c1].end : paths[c1].start;
                const NXOpen::Point3d& p2 = s2 ? paths[c2].start : paths[c2].end;
                const NXOpen::Point3d& p3 = s3 ? paths[c3].end : paths[c3].start;
                const NXOpen::Point3d& p4 = s4 ? paths[c4].start : paths[c4].end;

                // 计算当前连接的距离
                double currentDist = PointDistance(p1, p2) + PointDistance(p3, p4);

                // 计算交换后连接距离
                double newDist = PointDistance(p1, p3) + PointDistance(p2, p4);

                // 如果新的路径更短，则执行交换
                if (newDist < currentDist) {
                    // 反转中间路径
                    std::reverse(sequence.begin() + i + 1, sequence.begin() + j + 1);

                    // 翻转起点/终点标记
                    for (size_t k = i + 1; k <= j; k++) {
                        sequence[k].second = !sequence[k].second;
                    }

                    improved = true;
                }
            }
        }
    }
}

void SprayPathSequencer::solve()
{
    if (paths.empty()) {
        return;
    }

    int numPaths = static_cast<int>(paths.size());

    // 使用遗传算法优化
    // 参数：种群大小=60, 代数=150, 交叉率=0.85, 变异率=0.15, 精英数=3
    GeneticAlgorithm ga(this, startPathIndex, startFromPathStart, 60, 150, 0.85, 0.15, 3);

    // 记录优化前的初始距离（使用贪心算法的结果）
    auto greedySequence = greedyInitialSolution(
        (startPathIndex >= 0) ? startPathIndex : 0);
    initialDistance = calculatePathLength(greedySequence);

    // 执行遗传算法优化
    finalSequence = ga.solve();

    // 计算优化后的距离
    bestDistance = calculatePathLength(finalSequence);
}

std::vector<std::pair<int, bool>> SprayPathSequencer::getFinalSequence() const
{
    return finalSequence;
}

std::vector<NXOpen::TaggedObject*> SprayPathSequencer::getOptimizedCurves() const
{
    std::vector<NXOpen::TaggedObject*> optimizedCurves;
    optimizedCurves.reserve(finalSequence.size());

    // 根据优化后的序列顺序，提取曲线对象
    for (const auto& item : finalSequence) {
        int pathIdx = item.first;
        if (pathIdx >= 0 && pathIdx < (int)paths.size()) {
            tag_t curveTag = paths[pathIdx].tag;
            NXOpen::TaggedObject* curveObj = NXOpen::NXObjectManager::Get(curveTag);
            if (curveObj != nullptr) {
                optimizedCurves.push_back(curveObj);
            }
        }
    }

    return optimizedCurves;
}

double SprayPathSequencer::getBestDistance() const
{
    return bestDistance;
}

void SprayPathSequencer::printResult()
{
    if (finalSequence.empty()) {
        return;
    }

    // 使用固定输出目录
    std::string txtfileDir = LOG_OUTPUT_DIR;

    // 生成带时间戳的文件名，使用stringstream
    time_t now = time(0);
    struct tm timeinfo;
    localtime_s(&timeinfo, &now);

    std::stringstream ss;
    ss << "SprayPathSequence_"
        << (timeinfo.tm_year + 1900)
        << std::setfill('0') << std::setw(2) << (timeinfo.tm_mon + 1)
        << std::setfill('0') << std::setw(2) << timeinfo.tm_mday << "_"
        << std::setfill('0') << std::setw(2) << timeinfo.tm_hour
        << std::setfill('0') << std::setw(2) << timeinfo.tm_min
        << std::setfill('0') << std::setw(2) << timeinfo.tm_sec
        << ".txt";

    std::string fullPath = txtfileDir + "\\" + ss.str();

    // 打开文件
    logFile.open(fullPath, std::ios::out);
    if (!logFile.is_open()) {
        return;
    }

    // 写入数据
    logFile << "===================== 喷砂路径优化结果 =====================" << std::endl;
    logFile << "路径总数: " << paths.size() << std::endl;
    logFile << std::endl;

    // 优化前后对比
    logFile << "【优化前后对比】" << std::endl;
    logFile << "  优化前总空移距离: " << std::fixed << std::setprecision(2)
              << initialDistance << " mm" << std::endl;
    logFile << "  优化后总空移距离: " << std::fixed << std::setprecision(2)
              << bestDistance << " mm" << std::endl;

    double savedDistance = initialDistance - bestDistance;
    double optimizationRate = (initialDistance > 0) ? (savedDistance / initialDistance * 100.0) : 0.0;

    logFile << "  节省空移距离: " << std::fixed << std::setprecision(2)
              << savedDistance << " mm" << std::endl;
    logFile << "  优化比例: " << std::fixed << std::setprecision(2)
              << optimizationRate << " %" << std::endl;
    logFile << "===========================================================" << std::endl;
    logFile << std::endl;

    // 统计各个区域类型的路径数量
    int horizontalCount = 0;
    int convexCount = 0;
    int concaveCount = 0;
    int sidewallCount = 0;
    int unsortedCount = 0;

    for (const auto& path : paths) {
        switch (path.regionType) {
        case FaceRegionType::HORIZONTAL:
            horizontalCount++;
            break;
        case FaceRegionType::CONVEX:
            convexCount++;
            break;
        case FaceRegionType::CONCAVE:
            concaveCount++;
            break;
        case FaceRegionType::SIDEWALL:
            sidewallCount++;
            break;
        case FaceRegionType::UNSORTED:
            unsortedCount++;
            break;
        }
    }

    logFile << "路径类型分布:" << std::endl;
    if (horizontalCount > 0) logFile << "  水平区域: " << horizontalCount << std::endl;
    if (convexCount > 0) logFile << "  凸起弧面区域: " << convexCount << std::endl;
    if (concaveCount > 0) logFile << "  凹陷弧面区域: " << concaveCount << std::endl;
    if (sidewallCount > 0) logFile << "  侧面壁区域: " << sidewallCount << std::endl;
    if (unsortedCount > 0) logFile << "  未分类: " << unsortedCount << std::endl;
    logFile << std::endl;

    logFile << "===========================================================" << std::endl;
    logFile << "详细路径序列:" << std::endl;
    logFile << "===========================================================" << std::endl;

    for (size_t i = 0; i < finalSequence.size(); i++) {
        int pathIdx = finalSequence[i].first;
        bool isReversed = finalSequence[i].second;

        logFile << "序号: " << (i + 1) << std::endl;
        logFile << "  路径索引: " << pathIdx << std::endl;
        logFile << "  曲线Tag: " << paths[pathIdx].tag << std::endl;
        logFile << "  区域ID: " << paths[pathIdx].regionId << std::endl;
        logFile << "  执行方向: " << (isReversed ? "正向" : "反向") << std::endl;
        logFile << "  路径长度: " << paths[pathIdx].length << " mm" << std::endl;

        // 计算到下一条路径的空移距离
        if (i < finalSequence.size() - 1) {
            int nextPathIdx = finalSequence[i + 1].first;
            bool nextIsReversed = finalSequence[i + 1].second;

            const NXOpen::Point3d& currentEnd = isReversed ?
                paths[pathIdx].end : paths[pathIdx].start;
            const NXOpen::Point3d& nextStart = nextIsReversed ?
                paths[nextPathIdx].start : paths[nextPathIdx].end;

            double airMove = PointDistance(currentEnd, nextStart);
            logFile << "  空移到下一路径: " << airMove << " mm" << std::endl;
        }

        logFile << std::endl;
    }

    logFile << "===========================================================" << std::endl;
    logFile << "优化完成！" << std::endl;

    // 关闭文件
    logFile.close();
}

void SprayPathSequencer::createConnectionLines()
{
    if (finalSequence.empty() || finalSequence.size() < 2) {
        return;  // 至少需要两条路径才能创建连接线
    }
#ifdef _MYDEBUG
    std::string txtfileDir = LOG_OUTPUT_DIR;

    time_t now = time(0);
    struct tm timeinfo;
    localtime_s(&timeinfo, &now);

    std::stringstream ss;
    ss << "ConnectionLines_"
        << (timeinfo.tm_year + 1900)
        << std::setfill('0') << std::setw(2) << (timeinfo.tm_mon + 1)
        << std::setfill('0') << std::setw(2) << timeinfo.tm_mday << "_"
        << std::setfill('0') << std::setw(2) << timeinfo.tm_hour
        << std::setfill('0') << std::setw(2) << timeinfo.tm_min
        << std::setfill('0') << std::setw(2) << timeinfo.tm_sec
        << ".txt";

    std::string fullPath = txtfileDir + "\\" + ss.str();

    std::ofstream debugFile(fullPath, std::ios::out);
    if (!debugFile.is_open()) {
        return;  // 如果文件打开失败，继续执行但不记录日志
    }

    debugFile << "==================== 连接线创建详细日志 ====================" << std::endl;
    debugFile << "总路径数: " << paths.size() << std::endl;
    debugFile << "序列长度: " << finalSequence.size() << std::endl;
    debugFile << "连接线数量: " << (finalSequence.size() - 1) << std::endl;
    debugFile << "===========================================================" << std::endl;
    debugFile << std::endl;
#endif // _MYDEBUG


    // 遍历优化后的路径序列，创建相邻路径之间的连接线
    for (size_t i = 0; i < finalSequence.size() - 1; i++) {
        int pathIdx = finalSequence[i].first;
        int nextPathIdx = finalSequence[i + 1].first;
        bool isReversed = finalSequence[i].second;
        bool nextIsReversed = finalSequence[i + 1].second;

        // 获取当前路径的终点
        const NXOpen::Point3d& currentEnd = isReversed ?
            paths[pathIdx].end : paths[pathIdx].start;

        // 获取下一条路径的起点
        const NXOpen::Point3d& nextStart = nextIsReversed ?
            paths[nextPathIdx].start : paths[nextPathIdx].end;

        // 计算距离
        double distance = PointDistance(currentEnd, nextStart);
#ifdef _MYDEBUG
        // 写入日志
        debugFile << "【连接线 " << (i + 1) << "】" << std::endl;
        debugFile << "-------------------------------------------------------" << std::endl;
        debugFile << "起始路径信息:" << std::endl;
        debugFile << "  路径索引: " << pathIdx << std::endl;
        debugFile << "  曲线Tag: " << paths[pathIdx].tag << std::endl;
        debugFile << "  区域ID: " << paths[pathIdx].regionId << std::endl;
        debugFile << "  执行方向: " << (isReversed ? "正向 (start→end)" : "反向 (end→start)") << std::endl;
        debugFile << "  使用端点: " << (isReversed ? "end点" : "start点") << std::endl;
        debugFile << "  端点坐标: (" << std::fixed << std::setprecision(3)
            << currentEnd.X << ", " << currentEnd.Y << ", " << currentEnd.Z << ")" << std::endl;
        debugFile << std::endl;

        debugFile << "目标路径信息:" << std::endl;
        debugFile << "  路径索引: " << nextPathIdx << std::endl;
        debugFile << "  曲线Tag: " << paths[nextPathIdx].tag << std::endl;
        debugFile << "  区域ID: " << paths[nextPathIdx].regionId << std::endl;
        debugFile << "  执行方向: " << (nextIsReversed ? "正向 (start→end)" : "反向 (end→start)") << std::endl;
        debugFile << "  使用端点: " << (nextIsReversed ? "start点" : "end点") << std::endl;
        debugFile << "  端点坐标: (" << std::fixed << std::setprecision(3)
            << nextStart.X << ", " << nextStart.Y << ", " << nextStart.Z << ")" << std::endl;
        debugFile << std::endl;

        debugFile << "连接线信息:" << std::endl;
        debugFile << "  空移距离: " << std::fixed << std::setprecision(3) << distance << " mm" << std::endl;
#endif // _MYDEBUG

        // 使用UF_CURVE创建直线
        double startPoint[3] = { currentEnd.X, currentEnd.Y, currentEnd.Z };
        double endPoint[3] = { nextStart.X, nextStart.Y, nextStart.Z };

        CreateTempLine(startPoint, endPoint, 186);
#ifdef _MYDEBUG
        if (errorCode == 0 && lineTag != NULL_TAG) {
            // 可选：设置连接线的属性，例如颜色、图层等
            UF_OBJ_set_layer(lineTag, 88);
            debugFile << "  创建状态: 成功" << std::endl;
            debugFile << "  连接线Tag: " << lineTag << std::endl;
        }
        else {
            debugFile << "  创建状态: 失败 (错误码: " << errorCode << ")" << std::endl;
        }

        debugFile << "=======================================================" << std::endl;
        debugFile << std::endl;
    }

    debugFile << "==================== 端点使用统计 ====================" << std::endl;
    debugFile << std::endl;

    // 统计每个端点的使用情况
    std::map<std::string, std::vector<std::string>> endpointUsage;

    for (size_t i = 0; i < finalSequence.size(); i++) {
        int pathIdx = finalSequence[i].first;
        bool isReversed = finalSequence[i].second;

        // 入口端点
        const NXOpen::Point3d& entryPoint = isReversed ?
            paths[pathIdx].start : paths[pathIdx].end;

        // 出口端点
        const NXOpen::Point3d& exitPoint = isReversed ?
            paths[pathIdx].end : paths[pathIdx].start;

        std::stringstream entryKey, exitKey;
        entryKey << std::fixed << std::setprecision(3)
            << entryPoint.X << "," << entryPoint.Y << "," << entryPoint.Z;
        exitKey << std::fixed << std::setprecision(3)
            << exitPoint.X << "," << exitPoint.Y << "," << exitPoint.Z;

        std::stringstream usage;
        usage << "路径" << pathIdx << " (Tag:" << paths[pathIdx].tag << ") - "
            << (isReversed ? "入口(start)" : "入口(end)");
        endpointUsage[entryKey.str()].push_back(usage.str());

        usage.str("");
        usage << "路径" << pathIdx << " (Tag:" << paths[pathIdx].tag << ") - "
            << (isReversed ? "出口(end)" : "出口(start)");
        endpointUsage[exitKey.str()].push_back(usage.str());
    }

    debugFile << "【端点坐标使用详情】" << std::endl;
    for (const auto& kv : endpointUsage) {
        debugFile << "坐标: (" << kv.first << ")" << std::endl;
        debugFile << "  使用次数: " << kv.second.size() << std::endl;
        if (kv.second.size() > 2) {
            debugFile << "  ⚠️ 警告: 此端点被使用超过2次！" << std::endl;
        }
        for (const auto& usage : kv.second) {
            debugFile << "    - " << usage << std::endl;
        }
        debugFile << std::endl;
    }

    debugFile << "===========================================================" << std::endl;
    debugFile << "日志记录完成！" << std::endl;

    debugFile.close();
#endif // _MYDEBUG

    }
}