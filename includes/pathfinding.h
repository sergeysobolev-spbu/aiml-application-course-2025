#ifndef PATHFINDING_HDR
#define PATHFINDING_HDR

// Структура для представления точки на плоскости
typedef struct {
    double x;
    double y;
} Point;

// Структура для представления запрещенной зоны
typedef struct {
    Point center;
} ForbiddenZone;

#define MAX_ZONES 10
#define ZONE_RADIUS 1.0 // Радиус запрещенной зоны


void find_route(Point start, Point end, ForbiddenZone zones[], int zone_count);
int is_in_forbidden_zone(Point p, ForbiddenZone zones[], int zone_count);

#endif // PATHFINDING_HDR