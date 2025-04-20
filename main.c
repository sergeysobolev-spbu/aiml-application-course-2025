#include <stdio.h>
#include <stdbool.h>
#include <time.h>


#include "ant_colony.h"



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