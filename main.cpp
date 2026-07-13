#include <iostream>
#include <vector>
#include <limits>
#include <algorithm>
#include <queue>

#define INF std::numeric_limits<int>::max()

// 定义车辆结构体
struct Vehicle {
    int start, end;
    std::vector<int> path; // 路径
    int path_size;         // 路径长度

    Vehicle() : start(0), end(0), path_size(0) {}
};

// 定义流量区间结构体
struct FlowInterval {
    int interval_start;
    int interval_end;
    int used_capacity;
};

// 全局变量
int N, M; // 节点数和车辆数
std::vector<std::vector<int>> dist;
std::vector<std::vector<int>> capacity;
std::vector<std::vector<std::vector<FlowInterval>>> flow;
std::vector<Vehicle> vehicles;

// 检查是否可以通过
bool canPass(int u, int v, int time_start, int time_end, int max_capacity) {
    for(auto &interval : flow[u][v]) {
        if(!(time_end <= interval.interval_start || time_start >= interval.interval_end)) {
            if(interval.used_capacity >= max_capacity) {
                return false;
            }
        }
    }
    return true;
}

// 添加流量信息
void addFlow(int u, int v, int time_start, int time_end) {
    FlowInterval newInterval;
    newInterval.interval_start = time_start;
    newInterval.interval_end = time_end;
    newInterval.used_capacity = 1;
    flow[u][v].push_back(newInterval);
}

// 使用小顶堆优先队列按最早到达时间进行扩展
bool dijkstra(int start, int end, std::vector<int> &finalPath, int &travelTime) {
    static bool earliest[700][300]; 
    static int parent[700][300];     
    static int parentTime[700][300]; 

    for(int i = 0; i < N; i++){
        for(int t = 0; t < 300; t++){
            earliest[i][t] = false;
            parent[i][t] = -1;
            parentTime[i][t] = -1;
        }
    }

    // 小顶堆，元素为 (time, node)
    std::priority_queue<
        std::pair<int,int>,
        std::vector<std::pair<int,int>>,
        std::greater<std::pair<int,int>>
    > pq;

    earliest[start][0] = true;
    parent[start][0] = start;
    parentTime[start][0] = 0;
    pq.push({0, start});

    int foundTime = -1;
    while(!pq.empty()){
        auto [curT, u] = pq.top();
        pq.pop();
        if(u == end){
            foundTime = curT;
            break;
        }
        // 再次判断是否与 earliest[] 状态一致(可能已被更新)
        if(!earliest[u][curT]) continue;

        // 扩展到相邻节点
        for(int v = 0; v < N; v++){
            int cost = dist[u][v];
            if(cost <= 0) continue;  
            int arrivalT = curT + cost;
            if(arrivalT < 300 && !earliest[v][arrivalT]) {
                // 检查容量限制
                if(canPass(u, v, curT, arrivalT, capacity[u][v])) {
                    earliest[v][arrivalT] = true;
                    parent[v][arrivalT] = u;
                    parentTime[v][arrivalT] = curT;
                    pq.push({arrivalT, v});
                }
            }
        }
    }

    if(foundTime < 0) return false; // 无法到达
    travelTime = foundTime;

    // 回溯
    std::vector<int> path;
    int curNode = end, curT = foundTime;
    while(!(curNode == start && curT == 0)) {
        path.push_back(curNode);
        int pn = parent[curNode][curT];
        int pt = parentTime[curNode][curT];
        curNode = pn;
        curT = pt;
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    finalPath = path;
    return true;
}

int main() {
    std::cin >> N >> M;
    
    // 初始化距离和容量矩阵
    dist.assign(N, std::vector<int>(N, 0));
    capacity.assign(N, std::vector<int>(N, 0));
    flow.assign(N, std::vector<std::vector<FlowInterval>>(N, std::vector<FlowInterval>()));

    // 距离矩阵
    for(int i = 0; i < N; ++i)
        for(int j = 0; j < N; ++j)
            std::cin >> dist[i][j];

    // 容量矩阵
    for(int i = 0; i < N; ++i)
        for(int j = 0; j < N; ++j)
            std::cin >> capacity[i][j];

    // 读取车辆起点和终点
    vehicles.resize(M);
    for(int i = 0; i < M; ++i) {
        std::cin >> vehicles[i].start >> vehicles[i].end;
        vehicles[i].start--;
        vehicles[i].end--;
    }

    int totalTime = 0;
    for(int i = 0; i < M; ++i) {
        std::vector<int> path;
        int travelTime = 0;
        // 新 dijkstra
        if(dijkstra(vehicles[i].start, vehicles[i].end, path, travelTime)) {
            // 将搜索到的路径写入车辆
            int time_start = 0;
            for(int j = 1; j < (int)path.size(); ++j) {
                int u = path[j-1];
                int v = path[j];
                int time_end = time_start + dist[u][v];
                addFlow(u, v, time_start, time_end);
                time_start = time_end;
            }
            vehicles[i].path = path;
            vehicles[i].path_size = (int)path.size();
            totalTime += travelTime;
        }
    }

    for(auto &vehicle : vehicles) {
        for(auto node : vehicle.path)
            std::cout << (node + 1) << " ";
        std::cout << "\n";
    }

    std::cout << totalTime << "\n";

    return 0;
}