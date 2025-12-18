#include <iostream> 
#include <vector>   
#include <queue>   
#include <stack>  
#include <set>   
#include <algorithm>  
#include <cstdlib> 
#include <ctime>  
#include <climits> 
#include <chrono> 
#include <cmath> 
#include <cfloat>       

using namespace std;

// ======== Структуры для A* ========
struct Edge {
    int to;        // номер вершины
    double weight; // вес ребра
};

struct Node {
    int id;        // номер вершины
    double g;      // стоимость пути от старта
    double h;      // эвристическая оценка до цели
    double f;      // f = g + h

    // Меньший f — выше приоритет
    bool operator<(const Node& other) const {
        return f > other.f;
    }
};

// ======== DFS ========
vector<int> DFS(int start, int finish, vector<vector<int>>& graph) {
    int V = graph.size();
    vector<bool> visited(V, false);
    vector<int> path;
    stack<int> stk;
    stk.push(start);

    while (!stk.empty()) {
        int v = stk.top();
        stk.pop();

        if (!visited[v]) {
            visited[v] = true;
            path.push_back(v);

            // Если достигли цели, прерываем поиск
            if (v == finish) {
                break;
            }

            // Добавляем всех соседей в стек
            for (int u : graph[v]) {
                if (!visited[u]) {
                    stk.push(u);
                }
            }
        }
    }

    if (!path.empty() && path.back() != finish) {
        return {};
    }

    return path;
}

// ======== A* ========
// Эвристика
double heuristic(int x1, int y1, int x2, int y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// A* алгоритм
vector<int> aStar(
    const vector<vector<Edge>>& graph,  // список смежности
    const vector<pair<int, int>>& coords, // координаты вершин (x, y)
    int start,                          // начальная вершина
    int finish                          // конечная вершина
) {
    int n = graph.size();
    vector<double> gScore(n, DBL_MAX);  // стоимость пути от start
    vector<int> came_from(n, -1);       // для восстановления пути
    priority_queue<Node> openSet;

    // Инициализация
    gScore[start] = 0.0;
    openSet.push({ start, 0.0, heuristic(
        coords[start].first, coords[start].second,
        coords[finish].first, coords[finish].second
    ), 0.0 + heuristic(
        coords[start].first, coords[start].second,
        coords[finish].first, coords[finish].second
    ) });

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        // Проверяем, нашли ли цель
        if (current.id == finish) {
            // Восстанавливаем путь
            vector<int> path;
            int curr = finish;
            while (curr != start) {
                path.push_back(curr);
                curr = came_from[curr];
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
            return path;
        }

        // Проходим по соседям
        for (const Edge& edge : graph[current.id]) {
            double tentativeG = current.g + edge.weight;

            if (tentativeG < gScore[edge.to]) {
                // Обновляем лучший путь
                came_from[edge.to] = current.id;
                gScore[edge.to] = tentativeG;

                double h = heuristic(
                    coords[edge.to].first, coords[edge.to].second,
                    coords[finish].first, coords[finish].second
                );
                openSet.push({ edge.to, tentativeG, h, tentativeG + h });
            }
        }
    }

    return {};
}

// ======== Генерация графа ========
void generate_graph(int n, bool dense,
    vector<vector<int>>& adj_sparse,
    vector<vector<Edge>>& adj_weighted,
    vector<pair<int, int>>& coords) {

    adj_sparse.assign(n, vector<int>());
    adj_weighted.assign(n, vector<Edge>());
    coords.assign(n, { rand() % 1000, rand() % 1000 }); // случайные координаты для эвристики

    // Определяем количество рёбер
    int max_edges;
    if (dense) {
        // Для плотного графа: примерно n*(n-1)/4 рёбер (не полный, но плотный)
        max_edges = min(n * (n - 1) / 4, n * 10);
    }
    else {
        // Для разреженного графа: примерно 2*n рёбер
        max_edges = n * 2;
    }

    set<pair<int, int>> used;
    int edges_added = 0;

    // Добавляем рёбра, пока не достигнем нужного количества
    while (edges_added < max_edges) {
        int u = rand() % n;
        int v = rand() % n;

        if (u != v && !used.count({ u, v }) && !used.count({ v, u })) {
            used.insert({ u, v });
            used.insert({ v, u });

            // Для невзвешенного графа (DFS)
            adj_sparse[u].push_back(v);
            adj_sparse[v].push_back(u);

            // Для взвешенного графа (A*)
            double weight = 1.0 + (rand() % 100) / 10.0; // случайный вес от 1.0 до 10.0
            adj_weighted[u].push_back({ v, weight });
            adj_weighted[v].push_back({ u, weight });

            edges_added++;
        }

        // Защита от бесконечного цикла
        if (used.size() >= n * (n - 1)) {
            break;
        }
    }
}

// ======== Основная функция ========
int main() {
    setlocale(LC_CTYPE, "rus");
    srand(time(0));

    vector<int> sizes = { 10, 50, 100, 500, 1000, 5000, 10000 };
    for (int n : sizes) {
        cout << "\n==== Граф с " << n << " вершинами ====" << endl;

        vector<vector<int>> adj_sparse;
        vector<vector<Edge>> adj_weighted;
        vector<pair<int, int>> coords;

        // Разреженный граф
        generate_graph(n, false, adj_sparse, adj_weighted, coords);
        int start = rand() % n;
        int finish = rand() % n;
        cout << "\n-- Разреженный граф --" << endl;
        cout << "Старт: " << start << ", Финиш: " << finish << endl;

        auto t1 = chrono::high_resolution_clock::now();
        vector<int> path_dfs = DFS(start, finish, adj_sparse);
        auto t2 = chrono::high_resolution_clock::now();
        cout << "DFS: ";
        if (!path_dfs.empty()) {
            cout << "путь длины " << path_dfs.size();
        }
        else {
            cout << "путь не найден";
        }
        cout << ", время: " << chrono::duration<double, milli>(t2 - t1).count() << " ms\n";

        t1 = chrono::high_resolution_clock::now();
        vector<int> path_a = aStar(adj_weighted, coords, start, finish);
        t2 = chrono::high_resolution_clock::now();
        cout << "A*: ";
        if (!path_a.empty()) {
            cout << "путь длины " << path_a.size();
        }
        else {
            cout << "путь не найден";
        }
        cout << ", время: " << chrono::duration<double, milli>(t2 - t1).count() << " ms\n";

        // Плотный граф
        generate_graph(n, true, adj_sparse, adj_weighted, coords);
        start = rand() % n;
        finish = rand() % n;
        cout << "\n-- Плотный граф --" << endl;
        cout << "Старт: " << start << ", Финиш: " << finish << endl;

        t1 = chrono::high_resolution_clock::now();
        path_dfs = DFS(start, finish, adj_sparse);
        t2 = chrono::high_resolution_clock::now();
        cout << "DFS: ";
        if (!path_dfs.empty()) {
            cout << "путь длины " << path_dfs.size();
        }
        else {
            cout << "путь не найден";
        }
        cout << ", время: " << chrono::duration<double, milli>(t2 - t1).count() << " ms\n";

        t1 = chrono::high_resolution_clock::now();
        path_a = aStar(adj_weighted, coords, start, finish);
        t2 = chrono::high_resolution_clock::now();
        cout << "A*: ";
        if (!path_a.empty()) {
            cout << "путь длины " << path_a.size();
        }
        else {
            cout << "путь не найден";
        }
        cout << ", время: " << chrono::duration<double, milli>(t2 - t1).count() << " ms\n";
    }

    return 0;
}