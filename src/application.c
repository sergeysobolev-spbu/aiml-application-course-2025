#include "pathfinding.h"

int main()
{
    // Определяем стартовую и конечную точки
    Point start = {0.0, 0.0};
    Point end = {5.0, 5.0};

    // Определяем запрещенные зоны
    ForbiddenZone zones[MAX_ZONES] = {
        {{2.0, 2.0}},
        {{3.0, 3.0}},
        {{4.0, 4.0}}};

    int zone_count = 3; // Количество запрещенных зон

    // Запуск поиска маршрута
    find_route(start, end, zones, zone_count);

    return 0;
}
