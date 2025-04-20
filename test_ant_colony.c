/**
 * @file test_ant_colony.c
 * @brief Юнит-тесты для алгоритма муравьиной колонии.
 */

 #include <CUnit/CUnit.h>
 #include <CUnit/Basic.h>
 #include <stdbool.h>
 #include <math.h>
 #include <string.h>
 #include <stdlib.h>
 
 #include "ant_colony.h" // Подключаем ваш основной файл с алгоритмом
 
 /**
  * @brief Тестирование функции segments_intersect.
  */
 void test_segments_intersect(void) {
     Point p1 = {0, 0};
     Point p2 = {1, 1};
     Point p3 = {0, 1};
     Point p4 = {1, 0};
 
     CU_ASSERT_TRUE(segments_intersect(p1, p2, p3, p4));
 
     Point p5 = {0, 0};
     Point p6 = {1, 1};
     Point p7 = {1, 1};
     Point p8 = {2, 2};
 
     CU_ASSERT_FALSE(segments_intersect(p5, p6, p7, p8));
 }
 
 /**
  * @brief Тестирование функции is_path_clear.
  */
 void test_is_path_clear(void) {
     // Установим препятствия
     obstacle_count = 1;
     obstacles[0].points[0] = (Point){1, 1};
     obstacles[0].points[1] = (Point){1, 2};
     obstacles[0].points[2] = (Point){2, 2};
     obstacles[0].points[3] = (Point){2, 1};
 
     Point a = {0, 0};
     Point b = {3, 3};
 
     CU_ASSERT_TRUE(is_path_clear(a, b)); // Путь должен быть свободен
 
     Point c = {0, 0};
     Point d = {1, 1};
 
     CU_ASSERT_FALSE(is_path_clear(c, d)); // Путь должен быть заблокирован
 }
 
 /**
  * @brief Основная функция для запуска тестов.
  */
 int main() {
     CU_initialize_registry();
     
     CU_pSuite suite = CU_add_suite("Ant Colony Tests", NULL, NULL);
     
     CU_add_test(suite, "Test segments_intersect", test_segments_intersect);
     CU_add_test(suite, "Test is_path_clear", test_is_path_clear);
     
     CU_basic_set_mode(CU_BRM_VERBOSE);
     CU_basic_run_tests();
     
     CU_cleanup_registry();
     
     return 0;
 }
 