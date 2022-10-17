import pandas as pd
import folium

data = pd.read_csv('calles_de_medellin_con_acoso.csv', sep=';')
H_MEAN = data['harassmentRisk'].mean()
data['harassmentRisk'].fillna(H_MEAN, inplace=True)

def create_graph():
    graph = {}
    for i in data.index:
        origin_List = list(data['origin'][i][1:-1].split(','))
        origin_List[0], origin_List[1] = float(origin_List[1]), float(origin_List[0])
        origin_tuple = tuple(origin_List)

        destination_List = list(data['destination'][i][1:-1].split(','))
        destination_List[0], destination_List[1] = float(destination_List[1]), float(destination_List[0])
        destination_tuple  = tuple(destination_List)

        try:
            if data['oneway'][i]:
                graph[origin_tuple].update({destination_tuple: (data['length'][i], data['harassmentRisk'][i])})
                graph[destination_tuple].update({origin_tuple: (data['length'][i], data['harassmentRisk'][i])})
            else:
                graph[origin_tuple].update({destination_tuple: (data['length'][i], data['harassmentRisk'][i])})
        except KeyError:
            if data['oneway'][i]:
                if origin_tuple not in graph and destination_tuple not in graph:
                    graph[origin_tuple] = {destination_tuple: (data['length'][i], data['harassmentRisk'][i])}
                    graph[destination_tuple] = {origin_tuple: (data['length'][i], data['harassmentRisk'][i])}
                elif destination_tuple not in graph:
                    graph[origin_tuple].update({destination_tuple: (data['length'][i], data['harassmentRisk'][i])})
                    graph[destination_tuple] = {origin_tuple: (data['length'][i], data['harassmentRisk'][i])}
                else:
                    graph[destination_tuple].update({origin_tuple: (data['length'][i], data['harassmentRisk'][i])})
                    graph[origin_tuple] = {destination_tuple: (data['length'][i], data['harassmentRisk'][i])}
            else:
                if origin_tuple not in graph:
                    graph[origin_tuple] = {destination_tuple: (data['length'][i], data['harassmentRisk'][i])}
                else:
                    graph[origin_tuple].update({destination_tuple: (data['length'][i], data['harassmentRisk'][i])})

    return graph

grahp = create_graph()


def dijkstra(graph,start,goal):
    shortest_distance = {}
    predecessor = {}
    unseenNodes = graph
    infinity = 9999999
    path = []
    for node in unseenNodes:
        shortest_distance[node] = infinity
    shortest_distance[start] = 0

    while unseenNodes:
        minNode = None
        for node in unseenNodes:
            if minNode is None:
                minNode = node
            elif shortest_distance[node] < shortest_distance[minNode]:
                minNode = node

        for childNode, weight in graph[minNode].items():
            if weight[0] + shortest_distance[minNode] < shortest_distance[childNode]:
                shortest_distance[childNode] = weight[0] + shortest_distance[minNode]
                predecessor[childNode] = minNode
        unseenNodes.pop(minNode)

    currentNode = goal
    while currentNode != start:
        try:
            path.insert(0,currentNode)
            currentNode = predecessor[currentNode]
        except KeyError:
            return 'Camino imposible de obtener'
    path.insert(0,start)
    if shortest_distance[goal] != infinity:
        return path

path = dijkstra(grahp,(6.2734442, -75.5443961),(6.2094357, -75.5674348))

def create_map(paht):
    map = folium.Map(location=[paht[0][0], paht[0][1]], zoom_start=14)
    folium.PolyLine(paht, color='blue', weigth=15, opacity=0.8).add_to(map)
    map.save('index.html')

create_map(path)

