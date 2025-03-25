#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "pathfinding.h"


// Функция для проверки, попадает ли точка в запрещенную зону
int is_in_forbidden_zone(Point p, ForbiddenZone zones[], int zone_count) {
    for (int i = 0; i < zone_count; i++) {
        double distance = sqrt(pow(p.x - zones[i].center.x, 2) + pow(p.y - zones[i].center.y, 2));
        if (distance < ZONE_RADIUS) {
            return 1; // Точка находится в запрещенной зоне
        }
    }
    return 0; // Точка безопасна
}

// Функция для нахождения оптимального маршрута (упрощенная версия)
void find_route(Point start, Point end, ForbiddenZone zones[], int zone_count) {
    Point current = start;

    while (fabs(current.x - end.x) > 0.01 || fabs(current.y - end.y) > 0.01) {
        // Простейший алгоритм: двигаться к цели по оси X или Y
        if (current.x < end.x) {
            current.x += 0.1; // Движение вправо
        } else if (current.x > end.x) {
            current.x -= 0.1; // Движение влево
        }

        // Проверяем, не попали ли в запрещенную зону
        if (is_in_forbidden_zone(current, zones, zone_count)) {
            printf("Попали в запрещенную зону при движении по X: (%.2f, %.2f). Откат назад.\n", current.x, current.y);
            current.x = (current.x < end.x) ? current.x - 0.1 : current.x + 0.1;
        }

        if (current.y < end.y) {
            current.y += 0.1; // Движение вверх
        } else if (current.y > end.y) {
            current.y -= 0.1; // Движение вниз
        }

        // Проверяем, не попали ли в запрещенную зону
        if (is_in_forbidden_zone(current, zones, zone_count)) {
            printf("Попали в запрещенную зону при движении по Y: (%.2f, %.2f). Откат назад.\n", current.x, current.y);
            current.y = (current.y < end.y) ? current.y - 0.1 : current.y + 0.1;
        }

        printf("Текущая позиция: (%.2f, %.2f)\n", current.x, current.y);
    }

    printf("Маршрут завершен! Достигнута цель: (%.2f, %.2f)\n", end.x, end.y);
}
