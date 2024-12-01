import heapq

def dijkstra_new(graph, start):
    """
    Алгоритм Дейкстры с временной сложностью O((n + m) log n) с использованием двоичной кучи.

    :param graph: Список смежности (list of list of tuples). Каждый элемент - (сосед, вес).
    :param start: Индекс начальной вершины.
    :return: Список минимальных расстояний от начальной вершины до всех остальных вершин.
    """
    n = len(graph)
    distances = [float('inf')] * n
    distances[start] = 0

    # Очередь с приоритетом
    priority_queue = [(0, start)]  # (расстояние, вершина)

    while priority_queue:
        current_distance, current_vertex = heapq.heappop(priority_queue)

        # Если расстояние устарело, пропускаем
        if current_distance > distances[current_vertex]:
            continue

        # Обновляем расстояния до соседей
        for neighbor, weight in graph[current_vertex]:
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))

    return distances

# Примеры использования
if __name__ == "__main__":
    # Разреженный граф
    sparse_graph = [
        [(1, 10), (2, 5)],
        [(3, 1)],
        [(1, 3), (3, 9)],
        []
    ]
    print("Разреженный граф:")
    print(f"Минимальные расстояния от вершины 0: {dijkstra_new(sparse_graph, 0)}")

    # Плотный граф
    dense_graph = [
        [(1, 2), (2, 3), (3, 1)],
        [(0, 2), (2, 4), (3, 6)],
        [(0, 3), (1, 4), (3, 5)],
        [(0, 1), (1, 6), (2, 5)]
    ]
    print("\nПлотный граф:")
    print(f"Минимальные расстояния от вершины 0: {dijkstra_new(dense_graph, 0)}")

    # Ориентированный граф
    directed_graph = [
        [(1, 1), (2, 2)],
        [(3, 4)],
        [(3, 1)],
        []
    ]
    print("\nОриентированный граф:")
    print(f"Минимальные расстояния от вершины 0: {dijkstra_new(directed_graph, 0)}")
