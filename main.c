#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#define GRID_SIZE 20
#define MAX_POINTS 300
#define MAX_OBSTACLES 20
#define MAX_WAYPOINTS 10
#define INF 1e9
#define MAX_ANTS 50
#define MAX_ITERATIONS 100
#define ALPHA 1.0    // Влияние феромона
#define BETA 2.0     // Влияние эвристики (обратное расстояние)
#define RHO 0.1      // Коэффициент испарения феромона
#define Q 100.0      // Количество откладываемого феромона
#define PHEROMONE_INIT 0.1

typedef struct {
    int x, y;
} Point;

typedef struct {
    Point points[4];
    int id;
} Obstacle;

Point waypoints[MAX_WAYPOINTS];
int waypoint_count = 0;
Obstacle obstacles[MAX_OBSTACLES];
int obstacle_count = 0;
Point full_path[MAX_POINTS];
int full_path_length = 0;
double total_path_length = 0.0;

// Четырехмерная матрица феромонов
double pheromone[GRID_SIZE][GRID_SIZE];
// Динамическая четырехмерная матрица расстояний между точками
double ****distance_matrix;

char grid[GRID_SIZE][GRID_SIZE];

void init_grid() {
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            grid[y][x] = '.';
        }
    }
}

void print_grid() {
    printf("\nКарта маршрута (размер %dx%d):\n", GRID_SIZE, GRID_SIZE);
    printf("   ");
    for (int x = 0; x < GRID_SIZE; x++) printf("%d ", x);
    printf("\n");
    
    for (int y = 0; y < GRID_SIZE; y++) {
        printf("%d: ", y);
        for (int x = 0; x < GRID_SIZE; x++) {
            printf("%c ", grid[y][x]);
        }
        printf("\n");
    }
    printf("\n");
}

bool segments_intersect(Point p1, Point p2, Point p3, Point p4) {
    double d1 = (p4.x-p3.x)*(p1.y-p3.y) - (p4.y-p3.y)*(p1.x-p3.x);
    double d2 = (p4.x-p3.x)*(p2.y-p3.y) - (p4.y-p3.y)*(p2.x-p3.x);
    double d3 = (p2.x-p1.x)*(p3.y-p1.y) - (p2.y-p1.y)*(p3.x-p1.x);
    double d4 = (p2.x-p1.x)*(p4.y-p1.y) - (p2.y-p1.y)*(p4.x-p1.x);
    
    return (d1*d2 < 0) && (d3*d4 < 0);
}

bool is_path_clear(Point a, Point b) {
    for (int i = 0; i < obstacle_count; i++) {
        for (int j = 0; j < 4; j++) {
            Point p1 = obstacles[i].points[j];
            Point p2 = obstacles[i].points[(j+1)%4];
            if (segments_intersect(a, b, p1, p2)) {
                return false;
            }
        }
    }
    return true;
}

void draw_line(Point from, Point to, char symbol) {
    int dx = abs(to.x - from.x);
    int dy = abs(to.y - from.y);
    int sx = from.x < to.x ? 1 : -1;
    int sy = from.y < to.y ? 1 : -1;
    int err = (dx > dy ? dx : -dy) / 2;
    int e2;

    while (1) {
        if (from.x >= 0 && from.x < GRID_SIZE && 
            from.y >= 0 && from.y < GRID_SIZE) {
            if (grid[from.y][from.x] == '.' || grid[from.y][from.x] == symbol) {
                grid[from.y][from.x] = symbol;
            }
        }

        if (from.x == to.x && from.y == to.y) break;
        e2 = err;
        
        if (e2 > -dx) { err -= dy; from.x += sx; }
        if (e2 < dy) { err += dx; from.y += sy; }
    }
}

void init_pheromone() {
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            pheromone[y][x] = PHEROMONE_INIT;
        }
    }
}

void allocate_distance_matrix() {
    // Выделение памяти для четырехмерного массива
    distance_matrix = (double ****)malloc(GRID_SIZE * sizeof(double ***));
    for (int y1 = 0; y1 < GRID_SIZE; y1++) {
        distance_matrix[y1] = (double ***)malloc(GRID_SIZE * sizeof(double **));
        for (int x1 = 0; x1 < GRID_SIZE; x1++) {
            distance_matrix[y1][x1] = (double **)malloc(GRID_SIZE * sizeof(double *));
            for (int y2 = 0; y2 < GRID_SIZE; y2++) {
                distance_matrix[y1][x1][y2] = (double *)malloc(GRID_SIZE * sizeof(double));
            }
        }
    }
}

void free_distance_matrix() {
    // Освобождение памяти четырехмерного массива
    for (int y1 = 0; y1 < GRID_SIZE; y1++) {
        for (int x1 = 0; x1 < GRID_SIZE; x1++) {
            for (int y2 = 0; y2 < GRID_SIZE; y2++) {
                free(distance_matrix[y1][x1][y2]);
            }
            free(distance_matrix[y1][x1]);
        }
        free(distance_matrix[y1]);
    }
    free(distance_matrix);
}

void init_distance_matrix() {
    allocate_distance_matrix();
    
    for (int y1 = 0; y1 < GRID_SIZE; y1++) {
        for (int x1 = 0; x1 < GRID_SIZE; x1++) {
            for (int y2 = 0; y2 < GRID_SIZE; y2++) {
                for (int x2 = 0; x2 < GRID_SIZE; x2++) {
                    Point p1 = {x1, y1};
                    Point p2 = {x2, y2};
                    if (is_path_clear(p1, p2)) {
                        int dx = x2 - x1;
                        int dy = y2 - y1;
                        distance_matrix[y1][x1][y2][x2] = sqrt(dx*dx + dy*dy);
                    } else {
                        distance_matrix[y1][x1][y2][x2] = INF;
                    }
                }
            }
        }
    }
}

double calculate_probability(Point current, Point next, bool visited[GRID_SIZE][GRID_SIZE]) {
    if (visited[next.y][next.x] || !is_path_clear(current, next)) {
        return 0.0;
    }
    
    double tau = pheromone[next.y][next.x];
    double eta = 1.0 / distance_matrix[current.y][current.x][next.y][next.x];
    
    return pow(tau, ALPHA) * pow(eta, BETA);
}

void ant_colony_path(Point start, Point end, Point* path, int* path_length) {
    Point best_path[MAX_POINTS];
    int best_length = 0;
    double best_length_value = INF;
    
    init_pheromone();
    
    for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
        Point ant_paths[MAX_ANTS][MAX_POINTS];
        int ant_lengths[MAX_ANTS] = {0};
        double ant_length_values[MAX_ANTS] = {0};
        
        for (int ant = 0; ant < MAX_ANTS; ant++) {
            bool visited[GRID_SIZE][GRID_SIZE] = {false};
            Point current = start;
            visited[current.y][current.x] = true;
            ant_paths[ant][ant_lengths[ant]++] = current;
            
            while (current.x != end.x || current.y != end.y) {
                double probabilities[GRID_SIZE][GRID_SIZE] = {0};
                double total = 0.0;
                
                for (int y = 0; y < GRID_SIZE; y++) {
                    for (int x = 0; x < GRID_SIZE; x++) {
                        Point next = {x, y};
                        if (!visited[y][x] && (x != current.x || y != current.y)) {
                            probabilities[y][x] = calculate_probability(current, next, visited);
                            total += probabilities[y][x];
                        }
                    }
                }
                
                if (total > 0) {
                    for (int y = 0; y < GRID_SIZE; y++) {
                        for (int x = 0; x < GRID_SIZE; x++) {
                            probabilities[y][x] /= total;
                        }
                    }
                }
                
                double r = (double)rand() / RAND_MAX;
                double sum = 0.0;
                bool found = false;
                
                for (int y = 0; y < GRID_SIZE && !found; y++) {
                    for (int x = 0; x < GRID_SIZE && !found; x++) {
                        if (probabilities[y][x] > 0) {
                            sum += probabilities[y][x];
                            if (r <= sum) {
                                current = (Point){x, y};
                                visited[y][x] = true;
                                ant_paths[ant][ant_lengths[ant]++] = current;
                                found = true;
                            }
                        }
                    }
                }
                
                if (!found) break;
            }
            
            if (ant_lengths[ant] > 0 && 
                ant_paths[ant][ant_lengths[ant]-1].x == end.x && 
                ant_paths[ant][ant_lengths[ant]-1].y == end.y) {
                double len = 0;
                for (int i = 0; i < ant_lengths[ant]-1; i++) {
                    Point p1 = ant_paths[ant][i];
                    Point p2 = ant_paths[ant][i+1];
                    len += distance_matrix[p1.y][p1.x][p2.y][p2.x];
                }
                ant_length_values[ant] = len;
                
                if (len < best_length_value) {
                    best_length_value = len;
                    best_length = ant_lengths[ant];
                    memcpy(best_path, ant_paths[ant], best_length * sizeof(Point));
                }
            }
        }
        
        for (int y = 0; y < GRID_SIZE; y++) {
            for (int x = 0; x < GRID_SIZE; x++) {
                pheromone[y][x] *= (1.0 - RHO);
            }
        }
        
        for (int ant = 0; ant < MAX_ANTS; ant++) {
            if (ant_lengths[ant] > 0 && ant_length_values[ant] > 0) {
                double delta_pheromone = Q / ant_length_values[ant];
                
                for (int i = 0; i < ant_lengths[ant]-1; i++) {
                    Point p = ant_paths[ant][i];
                    pheromone[p.y][p.x] += delta_pheromone;
                }
            }
        }
    }
    
    *path_length = best_length;
    memcpy(path, best_path, best_length * sizeof(Point));
}

void build_full_path() {
    full_path_length = 0;
    total_path_length = 0.0;
    
    for (int i = 0; i <= waypoint_count; i++) {
        Point start_pt = waypoints[i];
        Point end_pt = waypoints[i+1];
        
        Point segment_path[MAX_POINTS];
        int segment_length = 0;
        
        ant_colony_path(start_pt, end_pt, segment_path, &segment_length);
        
        int start_idx = (i == 0) ? 0 : 1;
        for (int j = start_idx; j < segment_length; j++) {
            full_path[full_path_length++] = segment_path[j];
        }
        
        for (int j = 0; j < segment_length-1; j++) {
            Point p1 = segment_path[j];
            Point p2 = segment_path[j+1];
            total_path_length += distance_matrix[p1.y][p1.x][p2.y][p2.x];
        }
    }
    
    if (full_path_length > 0) {
        Point last_point = waypoints[waypoint_count+1];
        if (full_path[full_path_length-1].x != last_point.x || 
            full_path[full_path_length-1].y != last_point.y) {
            full_path[full_path_length++] = last_point;
        }
    }
}

void draw_elements() {
    init_grid();
    
    for (int i = 0; i < obstacle_count; i++) {
        for (int j = 0; j < 4; j++) {
            Point p1 = obstacles[i].points[j];
            Point p2 = obstacles[i].points[(j+1)%4];
            draw_line(p1, p2, '#');
        }
    }
    
    for (int i = 0; i < full_path_length - 1; i++) {
        draw_line(full_path[i], full_path[i+1], '@');
    }
    
    for (int i = 0; i <= waypoint_count + 1; i++) {
        Point p = waypoints[i];
        if (p.y >= 0 && p.y < GRID_SIZE && p.x >= 0 && p.x < GRID_SIZE) {
            if (i == 0) grid[p.y][p.x] = 'S';
            else if (i == waypoint_count + 1) grid[p.y][p.x] = 'E';
            else grid[p.y][p.x] = 'W';
        }
    }
}

void print_path_details() {
    printf("\nДетальная информация о маршруте:\n");
    printf("Всего точек в пути: %d\n", full_path_length);
    printf("Общая длина пути: %.2f единиц\n", total_path_length);
    
    printf("\nПоследовательность точек маршрута:\n");
    printf("Шаг | Координаты (x,y)\n");
    printf("----+----------------\n");
    
    for (int i = 0; i < full_path_length; i++) {
        printf("%3d | (%d, %d)", i+1, full_path[i].x, full_path[i].y);
        
        if (i == 0) printf(" (Старт)");
        else if (i == full_path_length - 1) printf(" (Финиш)");
        else {
            for (int j = 1; j <= waypoint_count; j++) {
                if (full_path[i].x == waypoints[j].x && full_path[i].y == waypoints[j].y) {
                    printf(" (Точка %d)", j);
                    break;
                }
            }
        }
        
        printf("\n");
    }
}

int main() {
    printf("запуск примера\n");
    srand(time(NULL));
    
    waypoints[0] = (Point){0, 0};
    waypoints[1] = (Point){2, 6};
    waypoints[2] = (Point){7, 0};
    waypoints[3] = (Point){9, 16};
    waypoints[4] = (Point){12, 8};
    waypoints[5] = (Point){13, 1};
    waypoints[6] = (Point){18, 2};
    waypoints[7] = (Point){13, 18};
    waypoints[8] = (Point){19, 19};
    waypoint_count = 7;

    obstacles[0].points[0] = (Point){7, 2};
    obstacles[0].points[1] = (Point){10, 2};
    obstacles[0].points[2] = (Point){10, 8};
    obstacles[0].points[3] = (Point){7, 8};
    obstacles[0].id = 1;
    
    obstacles[1].points[0] = (Point){12, 12};
    obstacles[1].points[1] = (Point){12, 16};
    obstacles[1].points[2] = (Point){16, 16};
    obstacles[1].points[3] = (Point){16, 12};
    obstacles[1].id = 2;
    
    obstacle_count = 2;

    init_distance_matrix();
    
    build_full_path();
    draw_elements();
    print_grid();
    
    printf("Легенда:\n");
    printf("S - Стартовая точка\n");
    printf("E - Конечная точка\n");
    printf("W - Промежуточные точки\n");
    printf("@ - Пройденный путь\n");
    printf("# - Препятствия\n\n");
    
    print_path_details();
    
    if (full_path_length > 0) {
        Point last = full_path[full_path_length-1];
        Point target = waypoints[waypoint_count+1];
        if (last.x == target.x && last.y == target.y) {
            printf("\nМаршрут успешно завершен в конечной точке (%d, %d)!\n", target.x, target.y);
        }
    }
    
    free_distance_matrix();
    
    return 0;
}