def dijkstra(graph, start):
    """
    Реализация алгоритма Дейкстры с временной сложностью O(n^2).

    :param graph: Матрица смежности графа (list of list). Если между вершинами i и j нет ребра, graph[i][j] = float('inf').
    :param start: Индекс начальной вершины.
    :return: Список минимальных расстояний от начальной вершины до всех остальных вершин.
    """
    n = len(graph)  # Количество вершин в графе

    # Минимальные расстояния от начальной вершины до остальных
    distances = [float('inf')] * n
    distances[start] = 0

    # Массив для отслеживания посещённых вершин
    visited = [False] * n

    for _ in range(n):
        # Шаг 1: Найти вершину с минимальным расстоянием из непосещённых
        min_distance = float('inf')
        u = -1

        for v in range(n):
            if not visited[v] and distances[v] < min_distance:
                min_distance = distances[v]
                u = v

        # Если вершина не найдена (все оставшиеся недостижимы), завершить
        if u == -1:
            break

        # Отметить вершину как посещённую
        visited[u] = True

        # Шаг 2: Обновить минимальные расстояния для соседей вершины u
        for v in range(n):
            if graph[u][v] != float('inf') and not visited[v]:
                distances[v] = min(distances[v], distances[u] + graph[u][v])

    return distances

# Пример использования
if __name__ == "__main__":
    # Пример графа (матрица смежности)
    graph = [
        [0, 1, 4, float('inf')],
        [float('inf'), 0, 2, 6],
        [float('inf'), float('inf'), 0, 3],
        [float('inf'), float('inf'), float('inf'), 0]
    ]

    start_vertex = 0  # Начальная вершина (A)
    distances = dijkstra(graph, start_vertex)

    print("Минимальные расстояния от вершины {}: {}".format(start_vertex, distances))
