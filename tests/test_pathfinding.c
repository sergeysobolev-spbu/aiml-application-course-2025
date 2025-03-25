#include <CUnit/CUnit.h>
#include <CUnit/Basic.h>
#include <math.h>

// Предполагается, что описание типов данных и функций находится в этом заголовочном файле
#include "pathfinding.h" 

#define ZONE_RADIUS 1.0

// Тестовые функции
void test_is_in_forbidden_zone() {
    ForbiddenZone zones[2] = {
        {{2.0, 2.0}},
        {{5.0, 5.0}}
    };

    Point p1 = {2.5, 2.5}; // Должен быть в зоне
    Point p2 = {4.0, 4.0}; // Должен быть вне зоны

    CU_ASSERT_TRUE(is_in_forbidden_zone(p1, zones, 2));
    CU_ASSERT_FALSE(is_in_forbidden_zone(p2, zones, 2));
}

void test_find_route() {
    Point start = {0.0, 0.0};
    Point end = {5.0, 5.0};

    ForbiddenZone zones[1] = {
        {{2.0, 2.0}} // Запрещенная зона
    };

    // Проверяем, что функция не застрянет в запрещенной зоне
    find_route(start, end, zones, 1);
    
    // Здесь можно добавить дополнительные проверки состояния,
    // если у вас есть возможность модифицировать функцию find_route
}

// Основная функция для запуска тестов
int main() {
    CU_initialize_registry();
    
    CU_pSuite suite = CU_add_suite("Pathfinding Tests", NULL, NULL);
    
    CU_add_test(suite, "Test is_in_forbidden_zone", test_is_in_forbidden_zone);
    CU_add_test(suite, "Test find_route", test_find_route);

    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();
    
    CU_cleanup_registry();
    return 0;
}
