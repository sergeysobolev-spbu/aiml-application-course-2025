from typing import List

# Функция для получения индекса состояния в Q-таблице
def get_index(coords: List[int]) -> int:
    return 64 * (coords[0] + 8 * coords[1]) + coords[2] + 8 * coords[3]

# Функция для получения координат цели и агента из индекса состояния
def get_coords(ind: int) -> List[int]:
    x_a = ind // 64 % 8
    y_a = ind // 64 // 8
    x_t = ind % 64 % 8
    y_t = ind % 64 // 8
    return [x_a, y_a, x_t, y_t]


# Проверка кодирования/декодирования
for i in range(8):
    for j in range(8):
        for k in range(8):
            for l in range(8):
                coords = [i, j, k ,l]
                print(coords, get_coords(get_index(coords)), get_index(coords))

                if coords != get_coords(get_index(coords)):
                    print('AAAAAAAAA!')
                    break