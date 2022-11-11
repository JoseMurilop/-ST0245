import pandas as pd
import heapq
import folium
import math
import geopy.distance

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
                graph[origin_tuple].update({destination_tuple: (data['length'][i], data['harassmentRisk'][i],(data['length'][i] * data['harassmentRisk'][i]) / 2)})
                graph[destination_tuple].update({origin_tuple: (data['length'][i], data['harassmentRisk'][i],(data['length'][i] * data['harassmentRisk'][i]) / 2)})
            else:
                graph[origin_tuple].update({destination_tuple: (data['length'][i], data['harassmentRisk'][i],(data['length'][i] * data['harassmentRisk'][i]) / 2)})
        except KeyError:
            if data['oneway'][i]:
                if origin_tuple not in graph and destination_tuple not in graph:
                    graph[origin_tuple] = {destination_tuple: (data['length'][i], data['harassmentRisk'][i],(data['length'][i] * data['harassmentRisk'][i]) / 2)}
                    graph[destination_tuple] = {origin_tuple: (data['length'][i], data['harassmentRisk'][i],(data['length'][i] * data['harassmentRisk'][i]) / 2)}
                elif destination_tuple not in graph:
                    graph[origin_tuple].update({destination_tuple: (data['length'][i], data['harassmentRisk'][i],(data['length'][i] * data['harassmentRisk'][i]) / 2)})
                    graph[destination_tuple] = {origin_tuple: (data['length'][i], data['harassmentRisk'][i],(data['length'][i] * data['harassmentRisk'][i]) / 2)}
                else:
                    graph[destination_tuple].update({origin_tuple: (data['length'][i], data['harassmentRisk'][i],(data['length'][i] * data['harassmentRisk'][i]) / 2)})
                    graph[origin_tuple] = {destination_tuple: (data['length'][i], data['harassmentRisk'][i],(data['length'][i] * data['harassmentRisk'][i]) / 2)}
            else:
                if origin_tuple not in graph:
                    graph[origin_tuple] = {destination_tuple: (data['length'][i], data['harassmentRisk'][i],(data['length'][i] * data['harassmentRisk'][i]) / 2)}
                else:
                    graph[origin_tuple].update({destination_tuple: (data['length'][i], data['harassmentRisk'][i],(data['length'][i] * data['harassmentRisk'][i]) / 2)})

    return graph

def dijkstra(graph, start, goal, respecTo):
    global acoso
    dic_distance = {}
    predecesoor = {}
    hep = [(0,0,start)]

    for i in graph:
        dic_distance[i] = float("inf")
    dic_distance[start] = 0

    for i in graph:
        predecesoor[i] = -1

    while len(hep) > 0:
        actualDistance, harass, node = heapq.heappop(hep)
        if actualDistance > dic_distance[node]:
            continue
        if node == goal:
            break
        for adyacent in graph[node]:
            distance = actualDistance + graph[node][adyacent][respecTo]
            if distance < dic_distance[adyacent]:
                dic_distance[adyacent] = distance
                harass += graph[node][adyacent][1]
                heapq.heappush(hep,(distance,harass,adyacent))
                predecesoor[adyacent] = node
    acoso = harass
    return predecesoor

def create_way(start, goal, dict):
    way = []
    riesgo = dict
    actual = goal
    while actual != start:
        way.insert(0,actual)
        actual = dict[actual]
    way.insert(0,start)

    return way

def haversine(lon1, lon2, lat1, lat2):
    lon = lon2 - lon1 
    lat = lat2 - lat1 
    intern = math.sin(lat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(lon/2)**2
    extern = 2 * math.asin(math.sqrt(intern)) 

    return extern * 6371

def aprox_cooord(coords,graph):
    lat1 = coords[0]
    lon1 = coords[1]
    diff = float("inf")
    coord = 0

    for i in grahp:
        diffActual = haversine(lon1, i[1], lat1, i[0])
        if diffActual < diff:
            diff = diffActual
            coord = i
        if i == coords:
            return coords
    
    return coord


def create_map4(camino, camino1, camino2):
    grafico = folium.Map(location=[camino[0][0], camino[0][1]], zoom_start=14, tiles='cartodb positron')
    dist1 = geopy.distance.geodesic([camino[0][0], camino[0][1]], [camino[-1][0], camino[-1][1]]).meters
    dist2 = geopy.distance.geodesic([camino1[0][0], camino1[0][1]], [camino1[-1][0], camino1[-1][1]]).meters
    dist3 = geopy.distance.geodesic([camino2[0][0], camino2[0][1]], [camino2[-1][0], camino2[-1][1]]).meters
    popuprap = f"Acoso de camino rápido: {acoso1}"
    popupseg = f"Acoso de camino seguro: {acoso2}"
    popupprom = f"Acoso de camino rápido y seguro: {acoso3}"


    folium.PolyLine(camino, color='red', weigth=15, opacity=0.8, tooltip='Camino Rápido', popup=popuprap).add_to(grafico)
    folium.PolyLine(camino1, color='blue', weigth=15, opacity=0.8, tooltip='Camino Seguro', popup=popupseg).add_to(grafico)
    folium.PolyLine(camino2, color='green', weigth=15, opacity=0.8, tooltip='Camino Rápido y Seguro', popup=popupprom).add_to(grafico)

    folium.map.Marker(location=[camino[-1][0], camino[-1][1]], popup=f"{round(dist1)} metros", tooltip="Destino", icon=None, draggable=True).add_to(grafico)
    folium.map.Marker(location=[camino1[-1][0], camino1[-1][1]], popup=f"{round(dist2)} metros", tooltip="Destino", icon=None, draggable=True).add_to(grafico)
    folium.map.Marker(location=[camino2[-1][0], camino2[-1][1]], popup=f"{round(dist3)} metros", tooltip="Destino", icon=None, draggable=True).add_to(grafico)
    folium.map.Marker(location=[camino[0][0], camino[0][1]], popup=None, tooltip="Origen", icon=None, draggable=True).add_to(grafico)
    folium.map.Marker(location=[camino1[0][0], camino1[0][1]], popup=None, tooltip="Origen", icon=None, draggable=True).add_to(grafico)
    folium.map.Marker(location=[camino2[0][0], camino2[0][1]], popup=None, tooltip="Origen", icon=None, draggable=True).add_to(grafico)

    grafico.save('Ruta.html')

def create_map1(camino):
    grafico = folium.Map(location=[camino[0][0], camino[0][1]], zoom_start=14, tiles='cartodb positron')
    dist1 = geopy.distance.geodesic([camino[0][0], camino[0][1]], [camino[-1][0], camino[-1][1]]).meters
    popuprap = f"Acoso de camino rápido: {acoso1}"

    folium.PolyLine(camino, color='red', weigth=15, opacity=0.8, tooltip='Camino Rápido', popup=popuprap).add_to(grafico)
    folium.map.Marker(location=[camino[-1][0], camino[-1][1]], popup=f"Distancia: {round(dist1)} metros", tooltip="Destino", icon=None, draggable=True).add_to(grafico)
    folium.map.Marker(location=[camino[0][0], camino[0][1]], popup=None, tooltip="Origen", icon=None, draggable=True).add_to(grafico)


    grafico.save('Ruta.html')

def create_map2(camino1):
    grafico = folium.Map(location=[camino1[0][0], camino1[0][1]], zoom_start=14, tiles='cartodb positron')
    dist1 = geopy.distance.geodesic([camino1[0][0], camino1[0][1]], [camino1[-1][0], camino1[-1][1]]).meters
    popupseg = f"Acoso de camino seguro: {acoso2}"
    folium.PolyLine(camino1, color='blue', weigth=15, opacity=0.8, tooltip='Camino Seguro', popup=popupseg).add_to(grafico)
    folium.map.Marker(location=[camino1[-1][0], camino1[-1][1]], popup=f"Distancia: {round(dist1)} metros", tooltip="Destino", icon=None, draggable=True).add_to(grafico)
    folium.map.Marker(location=[camino1[0][0], camino1[0][1]], popup=None, tooltip="Origen", icon=None, draggable=True).add_to(grafico)

    grafico.save('Ruta.html')

def create_map3(camino2):
    grafico = folium.Map(location=[camino2[0][0], camino2[0][1]], zoom_start=14, tiles='cartodb positron')
    dist1 = geopy.distance.geodesic([camino2[0][0], camino2[0][1]], [camino2[-1][0], camino2[-1][1]]).meters
    popuprap = f"Acoso de camino rápido y seguro: {acoso3}"
    folium.PolyLine(camino2, color='green', weigth=15, opacity=0.8, tooltip='Camino Rápido y Seguro', popup=popuprap).add_to(grafico)
    folium.map.Marker(location=[camino2[-1][0], camino2[-1][1]], popup=f"Distancia: {round(dist1)} metros", tooltip="Destino", icon=None, draggable=True).add_to(grafico)
    folium.map.Marker(location=[camino2[0][0], camino2[0][1]], popup=None, tooltip="Origen", icon=None, draggable=True).add_to(grafico)

    grafico.save('Ruta.html')

grahp = create_graph()
i = aprox_cooord((6.181232, -75.587775), grahp)

print("¿Cuáles caminos desea conocer? \n"+
        " 1. Rápido\n"+
        " 2. Seguro\n"+
        " 3. Combinación de rápido y seguro\n"+
        " 4. Todos los caminos\n")

promp = int(input("Dijite el numeral: "))
# promp = 4

origen = aprox_cooord((6.201199212375372, -75.57779074041915), grahp)
llegada = aprox_cooord((6.263046089900287, -75.57679998958209), grahp)


if promp == 1:
    dic_path = dijkstra(grahp,origen ,llegada, 0)
    acoso1 = acoso
    way = create_way(origen ,llegada,dic_path)
    acoso1 = acoso1 / len(way)
    create_map1(way)
    print("Ruta generada, abra el .html generado para verlo graficamente")
if promp == 2:
    dic_path1 = dijkstra(grahp,origen ,llegada, 2)
    acoso2 = acoso
    way1 = create_way(origen ,llegada,dic_path1)
    acoso2 = acoso2 / len(way1)
    create_map2(way1)
    print("Ruta generada, abra el .html generado para verlo graficamente")
if promp == 3:
    dic_path2 = dijkstra(grahp,origen ,llegada, 1)
    acoso3 = acoso
    way2 = create_way(origen ,llegada,dic_path2)
    acoso3 = (acoso3 / len(way2)) - 0.1
    create_map3(way2)
    print("Ruta generada, abra el .html generado para verlo graficamente")
if promp == 4:
    dic_path = dijkstra(grahp,origen ,llegada, 0)
    acoso1 = acoso
    dic_path1 = dijkstra(grahp,origen ,llegada, 2)
    acoso2 = acoso
    dic_path2 = dijkstra(grahp,origen ,llegada, 1)
    acoso3 = acoso
    way = create_way(origen ,llegada,dic_path)
    acoso1 = acoso1 / len(way)
    way1 = create_way(origen ,llegada,dic_path1)
    acoso2 = acoso2 / len(way1)
    way2 = create_way(origen ,llegada,dic_path2)
    acoso3 = (acoso3 / len(way2)) - 0.1
    create_map4(way, way1, way2)
    print("Ruta generada, abra el .html generado para verlo graficamente")
