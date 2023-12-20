from Graph import Graph
from random import randint
from math import sqrt, inf
import matplotlib.pyplot as plt

X_MAX = 800
Y_MAX= 600

class rectangle:
    def __init__(self, x1, y1, x2, y2):
        # Упорядочиваем углы, чтобы x1 <= x2 и y1 <= y2
        self.x1 = min(x1, x2)
        self.y1 = min(y1, y2)
        self.x2 = max(x1, x2)
        self.y2 = max(y1, y2)

        # Вычисляем остальные две вершины прямоугольника
        self.x3 = self.x1
        self.y3 = self.y2
        self.x4 = self.x2
        self.y4 = self.y1

        self.point_one = (self.x1, self.y1)
        self.point_three = (self.x2, self.y2)
        self.point_two = (self.x3, self.y3)
        self.point_four = (self.x4, self.y4)
        self.points = [self.point_one, self.point_two, self.point_three, self.point_four]

def is_point_inside_rectangle(p, a, b, c, d):
    def sign(p1, p2, p3):
        return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])

    d1 = sign(p, a, b)
    d2 = sign(p, b, c)
    d3 = sign(p, c, d)
    d4 = sign(p, d, a)

    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0) or (d4 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0) or (d4 > 0)

    return not (has_neg and has_pos)

def dot(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1]
def cross(v1, v2):
    return v1[0] * v2[1] - v2[0] * v1[1]
def segments_intersect(A, B, C, D):
    AB = (B[0] - A[0], B[1] - A[1])
    AC = (C[0] - A[0], C[1] - A[1])
    CD = (D[0] - C[0], D[1] - C[1])

    denom = cross(AB, CD)
    alpha = cross(AC, AB)
    beta = cross(AC, CD)

    if denom == 0:  # отрезки коллинеарны
        if alpha != 0:  # отрезки параллельны и не пересекаются
            return None
        t0 = dot(AC, AB) / dot(AB, AB)
        t1 = t0 + dot(CD, AB) / dot(AB, AB)
        t0, t1 = min(t0, t1), max(t0, t1)
        # Пересечение, если интервал [t0, t1] пересекается с интервалом [0, 1]
        if 0 <= t0 <= 1 or 0 <= t1 <= 1:
            return (
                (A[0] + t0 * AB[0], A[1] + t0 * AB[1]),
                (A[0] + t1 * AB[0], A[1] + t1 * AB[1])
            )
        else:
            return None
    else:
        if 0 <= alpha / denom <= 1 and 0 <= beta / denom <= 1:
            # Пересечение, если оба alpha/denom и beta/denom находятся в интервале [0, 1]
            t = beta / denom
            return (C[0] + t * CD[0], C[1] + t * CD[1])
        else:
            return None

def inter_rect(X,Y,rect):
    if segments_intersect(X,Y,rect.point_one,rect.point_two) or segments_intersect(X,Y,rect.point_two,rect.point_three) or segments_intersect(X, Y, rect.point_three, rect.point_four) or segments_intersect(X, Y, rect.point_one, rect.point_four):
        return True
def RandomSample():  # возвращает рандомные координаты точки
    x = randint(1, X_MAX)
    y = randint(1, Y_MAX)
    return (x,y)

def Length(X, Y):
    return sqrt((Y[0] - X[0]) ** 2 + (Y[1] - X[1]) ** 2)

def CollisionFree(X, Y, obstacles):
    for obstacle in obstacles:
        if (inter_rect(X,Y,obstacle)):
            return False
    return True

def Nearest(G, X,obstacles):
    min_distance = float('inf')
    nearest_vertex = None
    nearest_edge = None

    for vertex in G.get_vertices():
        distance = Length(X, vertex)
        if distance < min_distance and distance!=0.0:
            min_distance = distance
            nearest_vertex = vertex
            nearest_edge = None

    for edge in G.edges:
        v1, v2 = edge
        distance = DistanceToLineSegment(X, v1, v2)
        if distance < min_distance and distance!=0.0:
            min_distance = distance
            nearest_vertex = None
            nearest_edge = edge

    if nearest_edge is not None:
        # Разделение ребра и добавление новой вершины
        new_vertex = Steer(G,nearest_edge[0], nearest_edge[1],obstacles)
        G.add_vertex(new_vertex)
        G.add_edge(nearest_edge[0], new_vertex)
        G.add_edge(new_vertex, nearest_edge[1])
        return new_vertex
    else:
        return nearest_vertex

def DistanceToLineSegment(P, A, B):
    # Расстояние от точки P до отрезка AB
    def dot(v, w):
        return v[0] * w[0] + v[1] * w[1]

    def length(v):
        return sqrt(v[0] ** 2 + v[1] ** 2)

    def vector(p1, p2):
        return [p2[0] - p1[0], p2[1] - p1[1]]

    v = vector(A, P)
    w = vector(A, B)
    c1 = dot(v, w)
    if c1 <= 0:
        return length(vector(A, P))
    c2 = dot(w, w)
    if c2 <= c1:
        return length(vector(B, P))
    b = c1 / c2
    Pb = [A[0] + b * w[0], A[1] + b * w[1]]
    return length(vector(P, Pb))

def Steer(G,X, Y, obstacles ):
    scale_factor = randint(5,10)
    distance_xy = Length(X, Y)

    # Check if the length is zero
    if distance_xy == 0:
        return X

    dx = (abs(Y[0] - X[0]) / distance_xy) * scale_factor
    dy = (abs(Y[1] - X[1]) / distance_xy) * scale_factor
    Z = list(Y)  # Convert Y to a list so it can be modified
    while (
            not CollisionFree(X, Z, obstacles)
            and 0 < Z[0] < X_MAX
            and 0 < Z[1] < Y_MAX
            and not Z in G.get_vertices()
    ):
        Z[0] -= dx
        Z[1] -= dy
    return tuple(Z)  # Convert Z back to a tuple before returning

def Near(G, X, k):
    distances = [(vertex, Length(X, vertex)) for vertex in G.get_vertices()]
    distances.sort(key=lambda x: x[1])
    nearest_vertices = [vertex for vertex, _ in distances[:k] if vertex != X]
    return nearest_vertices

def ChooseParent(G, Qs, Qn, Qnear, obstacles):
    Qmin = Qn
    Cmin = Cost(G, Qn) + Length(Qn, Qs)

    for Q in Qnear:
        if CollisionFree(Q, Qs, obstacles):
            current_cost = Cost(G, Q) + Length(Q, Qs)
            if current_cost < Cmin:
                Qmin = Q
                Cmin = current_cost

    return Qmin

def Update(G, Qs, Qnear, Qinit, obstacles):
    for Q in Qnear:
        if Qs != Q:  # Check if Qs and Q are different
            if CollisionFree(Qs, Q, obstacles) and Cost(G, Qs) + Length(Qs, Q) < Cost(G, Q) and Qinit != Parent(G, Q):
                # Remove the old edge
                G.remove_edge(Parent(G, Q), Q)
                # Add the new edge
                G.add_edge(Qs, Q)

def Parent(G, V):
    for edge in G.edges:
        v1, v2 = edge
        if V == v2 and V != v1:
            return v1
    return None

def Cost(G, V):
    total_cost = 0
    current_vertex = V

    while True:
        parent = Parent(G, current_vertex)

        if parent is None:
            break
        total_cost += Length(parent, current_vertex)
        current_vertex = parent

    return total_cost

def shortest_path(graph, start_vertex, end_vertex):
    if len(graph.get_vertices()) < 2 or len(graph.get_adjacent(end_vertex)) < 1 or start_vertex == end_vertex:
        return []

    result = []
    distance = {}
    vertices = graph.get_vertices()

    for i in range(len(vertices)):
        distance[vertices[i]] = float('inf')

    parent = {}
    for i in range(len(vertices)):
        parent[vertices[i]] = -1

    distance[start_vertex] = 0

    while vertices:
        min_distance = distance[vertices[0]]
        min_distance_vertex = vertices[0]

        for i in range(len(vertices)):
            if distance[vertices[i]] < min_distance:
                min_distance = distance[vertices[i]]
                min_distance_vertex = vertices[i]

        vertices.remove(min_distance_vertex)

        if min_distance_vertex == end_vertex:
            vertex = end_vertex

            while vertex != start_vertex:
                result.insert(0, vertex)
                vertex = parent[vertex]
                if vertex == -1:
                    return []
            result.insert(0, start_vertex)

            return result

        adjacent_vertices = graph.get_adjacent(min_distance_vertex)
        for i in range(len(adjacent_vertices)):
            if distance[adjacent_vertices[i]] > distance[min_distance_vertex] + graph.get_edge_weight(min_distance_vertex, adjacent_vertices[i]):
                distance[adjacent_vertices[i]] = distance[min_distance_vertex] + graph.get_edge_weight(min_distance_vertex, adjacent_vertices[i])
                parent[adjacent_vertices[i]] = min_distance_vertex


def RRT_Star(N,k, Qinit, Qgoal,obstacles):
    G = Graph()
    G.add_vertex(Qinit)

    for i in range(1,N):

        if i%10 ==0:
            print("Iteration: ",i)
        Qrand = RandomSample()
        while Qrand in G.get_vertices():
            Qrand = RandomSample()
        Qn = Nearest(G, Qrand,obstacles)

        Qs = Steer(G,Qn, Qrand,obstacles)
        if Qs[0]<=0 or Qs[1]<=0 or Qs[0]>=X_MAX or Qs[1]>=Y_MAX:
            while Qs[0]<0 or Qs[1]<0 or Qs[0]>=X_MAX or Qs[1]>=Y_MAX:
                Qrand = RandomSample()
                Qn = Nearest(G, Qrand, obstacles)
                Qs = Steer(G, Qn, Qrand, obstacles)
        Qnear = Near(G, Qs, k)
        G.add_vertex(Qs)
        Qp = ChooseParent(G, Qs, Qn, Qnear,obstacles)
        G.add_edge(Qp, Qs)
        Update(G, Qs, Qnear,Qinit,obstacles)

    Qn = Near(G, Qgoal, k)
    G.add_vertex(Qgoal)
    for Q in Qn:
        Qnst=Nearest(G,Qgoal,obstacles)
        if (CollisionFree(Qnst,Qgoal,obstacles)):
            G.add_edge(Qnst, Qgoal)
            Update(G, Qgoal, Near(G, Qgoal, k), Qinit, obstacles)
            break
        if (CollisionFree(Q,Qgoal,obstacles)):
            G.add_edge(Q, Qgoal)
            Update(G, Qgoal, Near(G, Qgoal, k), Qinit, obstacles)
            break

    #path = []
    #for i in range(len(path_vertices) - 1):
        #path.append([path_vertices[i], path_vertices[i + 1]])
    #G.plot_edges(obstacles,path)
    #plt.show()
    path_vertices = shortest_path(G, Qinit, Qgoal)
    return path_vertices,G

