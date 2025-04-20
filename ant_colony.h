#ifndef ANT_COLONY_H
#define ANT_COLONY_H

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


extern Point waypoints[MAX_WAYPOINTS];
extern int waypoint_count;
extern Obstacle obstacles[MAX_OBSTACLES];
extern int obstacle_count;
extern Point full_path[MAX_POINTS];
extern int full_path_length;
extern double total_path_length;

// Четырехмерная матрица феромонов
extern double pheromone[GRID_SIZE][GRID_SIZE];
// Динамическая четырехмерная матрица расстояний между точками
extern double ****distance_matrix;


// Функция для инициализации феромонов
void init_pheromone();

// Функция для расчета вероятности перехода из одной точки в другую
double calculate_probability(Point current, Point next, bool visited[GRID_SIZE][GRID_SIZE]);

// Функция для расчета расстояния между двумя точками
double distance(Point a, Point b);

// Основная функция алгоритма муравьиной колонии для нахождения пути
void ant_colony_path(Point start, Point end, Point* path, int* path_length);

#endif // ANT_COLONY_H
