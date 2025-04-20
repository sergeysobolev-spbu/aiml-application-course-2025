#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

// Константы
#define GRID_SIZE 20
#define MAX_POINTS 500
#define MAX_OBSTACLES 20
#define MAX_WAYPOINTS 10
#define INF 1e9

// Параметры муравьиного алгоритма
#define ANTS_COUNT 20      // Уменьшено для оптимизации
#define ITERATIONS 50    // Уменьшено для оптимизации
#define ALPHA 1.0
#define BETA 3.0
#define RHO 0.3
#define Q 100.0
#define PHEROMONE_INIT 0.1
#define MAX_NEIGHBORS 8

// Структуры данных
typedef struct { int x, y; } Point;
typedef struct { Point points[4]; int id; } Obstacle;

// Глобальные переменные
Point waypoints[MAX_WAYPOINTS];
int waypoint_count = 0;
Obstacle obstacles[MAX_OBSTACLES];
int obstacle_count = 0;
Point full_path[MAX_POINTS];
int full_path_length = 0;
double total_path_length = 0.0;
char grid[GRID_SIZE][GRID_SIZE];

// Матрицы для муравьиного алгоритма
double pheromone[GRID_SIZE][GRID_SIZE];
double heuristic[GRID_SIZE][GRID_SIZE];
double probability[GRID_SIZE][GRID_SIZE];

// Прототипы функций
void init_grid();
void print_grid();
bool segments_intersect(Point p1, Point p2, Point p3, Point p4);
bool is_path_clear(Point a, Point b);
void draw_line(Point from, Point to, char symbol);
void init_aco();
void calculate_probabilities(Point current, Point target);
Point select_next_point(Point current);
void update_pheromones(Point* path, int path_length, double path_length_inv);
void find_path_between_points(Point start, Point end, Point* path, int* path_length);
void build_full_path();
void draw_elements();
void print_path_details();
void optimize_path_order(int* order);
void swap_int(int* a, int* b);

int main() {
    srand(time(NULL));
    
    // Инициализация данных
    waypoints[0] = (Point){0, 0};   // Старт
    waypoints[1] = (Point){2, 6};   // Точка 1
    waypoints[2] = (Point){3, 0};   // Точка 2
    waypoints[3] = (Point){9, 16};  // Точка 3
    waypoints[4] = (Point){12, 8};  // Точка 4
    waypoints[5] = (Point){13, 1};  // Точка 5
    waypoints[6] = (Point){18, 2};  // Точка 6
    waypoints[7] = (Point){13, 18}; // Точка 7
    waypoints[8] = (Point){19, 19}; // Финиш
    waypoint_count = 7;

    obstacles[0] = (Obstacle){{{7, 2}, {10, 2}, {10, 8}, {7, 8}}, 1};
    obstacles[1] = (Obstacle){{{12, 12}, {12, 16}, {16, 16}, {16, 12}}, 2};
    obstacles[2] = (Obstacle){{{15, 3}, {18, 3}, {18, 6}, {15, 6}}, 3};
    obstacle_count = 3;

    // Построение и отображение маршрута
    build_full_path();
    draw_elements();
    print_grid();
    print_path_details();

    return 0;
}

// Функция обмена значений
void swap_int(int* a, int* b) {
    int temp = *a;
    *a = *b;
    *b = temp;
}

// Инициализация сетки
void init_grid() {
    memset(grid, '.', sizeof(grid));
}

// Вывод сетки
void print_grid() {
    printf("\nКарта маршрута (размер %dx%d):\n", GRID_SIZE, GRID_SIZE);
    printf("   ");
    for (int x = 0; x < GRID_SIZE; x++) printf("%2d", x);
    printf("\n");
    
    for (int y = 0; y < GRID_SIZE; y++) {
        printf("%2d:", y);
        for (int x = 0; x < GRID_SIZE; x++) {
            printf(" %c", grid[y][x]);
        }
        printf("\n");
    }
}

// Проверка пересечения отрезков (оптимизированная)
bool segments_intersect(Point p1, Point p2, Point p3, Point p4) {
    int d1 = (p4.x-p3.x)*(p1.y-p3.y) - (p4.y-p3.y)*(p1.x-p3.x);
    int d2 = (p4.x-p3.x)*(p2.y-p3.y) - (p4.y-p3.y)*(p2.x-p3.x);
    if ((d1 > 0 && d2 > 0) || (d1 < 0 && d2 < 0)) return false;
    
    int d3 = (p2.x-p1.x)*(p3.y-p1.y) - (p2.y-p1.y)*(p3.x-p1.x);
    int d4 = (p2.x-p1.x)*(p4.y-p1.y) - (p2.y-p1.y)*(p4.x-p1.x);
    return !((d3 > 0 && d4 > 0) || (d3 < 0 && d4 < 0));
}

// Проверка свободен ли путь между точками
bool is_path_clear(Point a, Point b) {
    for (int i = 0; i < obstacle_count; i++) {
        for (int j = 0; j < 4; j++) {
            if (segments_intersect(a, b, obstacles[i].points[j], obstacles[i].points[(j+1)%4])) {
                return false;
            }
        }
    }
    return true;
}

// Алгоритм Брезенхема для рисования линии
void draw_line(Point from, Point to, char symbol) {
    int dx = abs(to.x - from.x), sx = from.x < to.x ? 1 : -1;
    int dy = -abs(to.y - from.y), sy = from.y < to.y ? 1 : -1;
    int err = dx + dy, e2;

    while (1) {
        if (from.x >= 0 && from.x < GRID_SIZE && from.y >= 0 && from.y < GRID_SIZE) {
            if (grid[from.y][from.x] == '.' || grid[from.y][from.x] == symbol) {
                grid[from.y][from.x] = symbol;
            }
        }

        if (from.x == to.x && from.y == to.y) break;
        e2 = 2 * err;
        
        if (e2 >= dy) { err += dy; from.x += sx; }
        if (e2 <= dx) { err += dx; from.y += sy; }
    }
}

// Инициализация феромонов
void init_aco() {
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            pheromone[y][x] = PHEROMONE_INIT;
            heuristic[y][x] = 1.0;
        }
    }
}

// Оптимизированный расчет вероятностей
void calculate_probabilities(Point current, Point target) {
    double sum = 0.0;
    int dx = target.x - current.x;
    int dy = target.y - current.y;
    double base_dist = sqrt(dx*dx + dy*dy) + 0.1;
    
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            if (x == current.x && y == current.y) {
                probability[y][x] = 0.0;
                continue;
            }
            
            if (!is_path_clear(current, (Point){x, y})) {
                probability[y][x] = 0.0;
            } else {
                dx = target.x - x;
                dy = target.y - y;
                heuristic[y][x] = 1.0 / (sqrt(dx*dx + dy*dy) + 0.1);
                probability[y][x] = pow(pheromone[y][x], ALPHA) * pow(heuristic[y][x], BETA);
                sum += probability[y][x];
            }
        }
    }
    
    // Нормализация
    if (sum > 0) {
        double inv_sum = 1.0 / sum;
        for (int y = 0; y < GRID_SIZE; y++) {
            for (int x = 0; x < GRID_SIZE; x++) {
                probability[y][x] *= inv_sum;
            }
        }
    }
}

// Выбор следующей точки
Point select_next_point(Point current) {
    double r = (double)rand() / RAND_MAX;
    double cumulative_prob = 0.0;
    
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            cumulative_prob += probability[y][x];
            if (r <= cumulative_prob) {
                return (Point){x, y};
            }
        }
    }
    return current;
}

// Обновление феромонов
void update_pheromones(Point* path, int path_length, double path_length_inv) {
    double delta = path_length_inv * Q;
    for (int i = 0; i < path_length - 1; i++) {
        Point p1 = path[i], p2 = path[i+1];
        pheromone[p1.y][p1.x] = (1.0 - RHO) * pheromone[p1.y][p1.x] + RHO * delta;
        pheromone[p2.y][p2.x] = (1.0 - RHO) * pheromone[p2.y][p2.x] + RHO * delta;
    }
}

// Поиск пути между точками с ACO
void find_path_between_points(Point start, Point end, Point* path, int* path_length) {
    init_aco();
    Point best_path[MAX_POINTS];
    int best_path_length = 0;
    double best_path_length_value = INF;
    
    const Point neighbors[8] = {
        {-1,-1}, {0,-1}, {1,-1}, {-1,0}, {1,0}, {-1,1}, {0,1}, {1,1}
    };

    for (int iter = 0; iter < ITERATIONS; iter++) {
        for (int ant = 0; ant < ANTS_COUNT; ant++) {
            Point current_path[MAX_POINTS];
            int current_path_length = 0;
            Point current = start;
            current_path[current_path_length++] = current;
            
            while (!(current.x == end.x && current.y == end.y) && current_path_length < MAX_POINTS) {
                calculate_probabilities(current, end);
                Point next = select_next_point(current);
                
                // Проверка границ и зацикливания
                if (next.x < 0 || next.x >= GRID_SIZE || next.y < 0 || next.y >= GRID_SIZE ||
                    !is_path_clear(current, next)) {
                    // Выбор случайного соседа
                    bool found = false;
                    for (int i = 0; i < 8; i++) {
                        Point neighbor = {current.x + neighbors[i].x, current.y + neighbors[i].y};
                        if (neighbor.x >= 0 && neighbor.x < GRID_SIZE && neighbor.y >= 0 && neighbor.y < GRID_SIZE &&
                            is_path_clear(current, neighbor)) {
                            next = neighbor;
                            found = true;
                            break;
                        }
                    }
                    if (!found) break;
                }
                
                current = next;
                current_path[current_path_length++] = current;
            }
            
            if (current_path[current_path_length-1].x == end.x && current_path[current_path_length-1].y == end.y) {
                double current_length = 0.0;
                for (int i = 0; i < current_path_length - 1; i++) {
                    int dx = current_path[i+1].x - current_path[i].x;
                    int dy = current_path[i+1].y - current_path[i].y;
                    current_length += sqrt(dx*dx + dy*dy);
                }
                
                update_pheromones(current_path, current_path_length, 1.0 / current_length);
                
                if (current_length < best_path_length_value) {
                    best_path_length_value = current_length;
                    best_path_length = current_path_length;
                    memcpy(best_path, current_path, best_path_length * sizeof(Point));
                }
            }
        }
    }
    
    if (best_path_length > 0) {
        *path_length = best_path_length;
        memcpy(path, best_path, best_path_length * sizeof(Point));
    } else {
        path[0] = start;
        path[1] = end;
        *path_length = 2;
    }
}

// Оптимизация порядка посещения точек
void optimize_path_order(int* order) {
    // Простая эвристика: сортировка по расстоянию от старта
    double distances[MAX_WAYPOINTS+2];
    for (int i = 1; i <= waypoint_count; i++) {
        int dx = waypoints[i].x - waypoints[0].x;
        int dy = waypoints[i].y - waypoints[0].y;
        distances[i] = sqrt(dx*dx + dy*dy);
    }
    
    // Пузырьковая сортировка (для небольших waypoint_count достаточно)
    for (int i = 1; i < waypoint_count; i++) {
        for (int j = i+1; j <= waypoint_count; j++) {
            if (distances[order[i]] > distances[order[j]]) {
                swap_int(&order[i], &order[j]);
            }
        }
    }
}

// Построение полного маршрута
void build_full_path() {
    int order[MAX_WAYPOINTS+2] = {0};
    for (int i = 1; i <= waypoint_count; i++) order[i] = i;
    order[waypoint_count+1] = waypoint_count+1;
    
    optimize_path_order(order);

    double min_total_length = INF;
    Point best_path[MAX_POINTS];
    int best_path_length = 0;
    
    // Ограничим число перестановок для производительности
    const int max_permutations = waypoint_count > 5 ? 5 : waypoint_count;
    
    for (int p = 0; p < max_permutations; p++) {
        if (p > 0 && waypoint_count > 1) {
            // Перемешиваем случайным образом
            int i = 1 + rand() % waypoint_count;
            int j = 1 + rand() % waypoint_count;
            swap_int(&order[i], &order[j]);
        }
        
        Point current_path[MAX_POINTS];
        int current_length = 0;
        double current_total = 0.0;
        
        for (int i = 0; i <= waypoint_count; i++) {
            Point segment[MAX_POINTS];
            int segment_length = 0;
            
            find_path_between_points(waypoints[order[i]], waypoints[order[i+1]], segment, &segment_length);
            
            int start_idx = (i == 0) ? 0 : 1;
            for (int j = start_idx; j < segment_length && current_length < MAX_POINTS; j++) {
                current_path[current_length++] = segment[j];
            }
            
            for (int j = 0; j < segment_length - 1; j++) {
                int dx = segment[j+1].x - segment[j].x;
                int dy = segment[j+1].y - segment[j].y;
                current_total += sqrt(dx*dx + dy*dy);
            }
        }
        
        if (current_total < min_total_length) {
            min_total_length = current_total;
            best_path_length = current_length;
            memcpy(best_path, current_path, current_length * sizeof(Point));
        }
    }
    
    full_path_length = best_path_length;
    memcpy(full_path, best_path, best_path_length * sizeof(Point));
    total_path_length = min_total_length;
}

// Визуализация элементов
void draw_elements() {
    init_grid();
    
    // Рисуем препятствия
    for (int i = 0; i < obstacle_count; i++) {
        for (int j = 0; j < 4; j++) {
            draw_line(obstacles[i].points[j], obstacles[i].points[(j+1)%4], '#');
        }
    }
    
    // Рисуем путь
    for (int i = 0; i < full_path_length - 1; i++) {
        draw_line(full_path[i], full_path[i+1], '@');
    }
    
    // Рисуем точки маршрута
    for (int i = 0; i <= waypoint_count + 1; i++) {
        Point p = waypoints[i];
        if (p.x >= 0 && p.x < GRID_SIZE && p.y >= 0 && p.y < GRID_SIZE) {
            grid[p.y][p.x] = (i == 0) ? 'S' : (i == waypoint_count + 1) ? 'E' : 'W';
        }
    }
}

// Вывод информации о маршруте
void print_path_details() {
    printf("\nДетальная информация о маршруте:\n");
    printf("Всего точек в пути: %d\n", full_path_length);
    printf("Общая длина пути: %.2f единиц\n", total_path_length);
    
    printf("\nПоследовательность точек маршрута:\n");
    printf("Шаг | Координаты (x,y) | Тип\n");
    printf("----+------------------+----------\n");
    
    for (int i = 0; i < full_path_length; i++) {
        printf("%3d | (%2d, %2d)      | ", i+1, full_path[i].x, full_path[i].y);
        
        if (i == 0) printf("Старт");
        else if (i == full_path_length - 1) printf("Финиш");
        else {
            bool is_waypoint = false;
            for (int j = 1; j <= waypoint_count; j++) {
                if (full_path[i].x == waypoints[j].x && full_path[i].y == waypoints[j].y) {
                    printf("Точка %d", j);
                    is_waypoint = true;
                    break;
                }
            }
            if (!is_waypoint) printf("Путь");
        }
        printf("\n");
    }
    
    // Проверка посещения всех точек
    bool all_visited = true;
    for (int i = 1; i <= waypoint_count; i++) {
        bool visited = false;
        for (int j = 0; j < full_path_length; j++) {
            if (full_path[j].x == waypoints[i].x && full_path[j].y == waypoints[i].y) {
                visited = true;
                break;
            }
        }
        if (!visited) {
            all_visited = false;
            printf("\nПРЕДУПРЕЖДЕНИЕ: Точка %d (%d, %d) не посещена!\n", 
                   i, waypoints[i].x, waypoints[i].y);
        }
    }
    
    if (all_visited) {
        printf("\nВсе промежуточные точки посещены успешно!\n");
    }
}
