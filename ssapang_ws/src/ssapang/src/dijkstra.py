#!/usr/bin/env python3
import rospy
from ssapang.msg import Locations, Coordinate, Move
import time

import heapq

# def dijkstra(graph, start_node, destination_node):
def dijkstra(graph, start_node, destination_node, cantGoNode=None):

    # 시작 노드로부터의 거리 초기화
    dist = {node: float('inf') for node in graph}
    dist[start_node] = 0

    # 시작 노드를 큐에 삽입
    pq = [(0, start_node)]

    # 최단 경로를 저장할 딕셔너리 초기화
    prev = {node: None for node in graph}

    while pq:
        # 현재 가장 짧은 거리를 가진 노드 선택
        curr_dist, curr_node = heapq.heappop(pq)
        # 이미 처리된 노드라면 건너뜀
        if curr_dist > dist[curr_node]:
            continue
        # 현재 노드와 연결된 모든 노드를 순회
        for next_node in graph[curr_node]:
            if cantGoNode!=None: #만약 지나면 안되는 노드가 있다면 통과
                if next_node == cantGoNode:
                    continue
            # 새로운 거리 계산
            new_dist = curr_dist + 1
            # 새로운 거리가 더 짧으면 업데이트
            if new_dist < dist[next_node]:
                dist[next_node] = new_dist
                prev[next_node] = [curr_node, graph.get(curr_node).get(next_node)]
                heapq.heappush(pq, (new_dist, next_node))
    # 도착 노드까지의 최단 경로 출력
    path = [[destination_node, graph.get(destination_node).get(destination_node)]]

    while path[-1][0] != start_node:
        path.append(prev[path[-1][0]])

    path.reverse()
    return path

def addCoordinates(arr):
    last = len(arr)-1
    for i in range(last):
        arr[i].append(node.get(arr[i+1][0]))
    arr[last].append([-100, 100])
    arr[last][1] = arr[last-1][1]
    print(arr)
    return arr

# 예시 그래프
graph = {

    'BS0101': {},
    'BS0102': {'B0102': -90},
    'BS0103': {'B0103': -90},
    'BS0104': {'B0104': -90},
    'BS0105': {'B0105': -90},
    'BS0106': {'B0106': -90},
    'BS0107': {'B0107': -90},
    'BS0108': {},
    'BS0109': {'B0109': -90},
    'BS0110': {'B0110': -90},
    'BS0111': {'B0111': -90},
    'BS0112': {'B0112': -90},
    'BS0113': {'B0113': -90},
    'BS0114': {'B0114': -90},
    'BS0115': {},

    'B0101': {'B0102': 0, 'BS0101': 90},
    'B0102': {'B0103': 0, 'B0202': -90},
    'B0103': {'B0104': 0},
    'B0104': {'B0105': 0, 'B0204': -90},
    'B0105': {'B0106': 0},
    'B0106': {'B0107': 0, 'B0206': -90},
    'B0107': {'B0207': -90},
    'B0108': {'BS0108': 90},
    'B0109': {'B0209': -90},
    'B0110': {'B0109': 180, 'B0210': -90},
    'B0111': {'B0110': 180},
    'B0112': {'B0111': 180, 'B0212': -90},
    'B0113': {'B0112': 180},
    'B0114': {'B0113': 180, 'B0214': -90},
    'B0115': {'B0114': 180, 'BS0115': 90},

    'B0201': {'B0101': 90},
    'B0202': {'BP0101': -90, 'B0201': 180},
    'B0203': {'B0202': 180},
    'B0204': {'BP0103': -90, 'B0203': 180},
    'B0205': {'B0204': 180},
    'B0206': {'BP0105': -90, 'B0205': 180},
    'B0207': {'BP0106': -90, 'B0206': 180},
    'B0208': {'B0108': 90, 'B0207': 180, 'B0209': 0},
    'B0209': {'BP0107': -90, 'B0210': 0},
    'B0210': {'BP0108': -90, 'B0211': 0},
    'B0211': {'B0212': 0},
    'B0212': {'BP0110': -90, 'B0213': 0},
    'B0213': {'B0214': 0},
    'B0214': {'BP0112': -90, 'B0215': 0},
    'B0215': {'B0115': 90},

    'BP0101': {'BP0201': -90},
    'BP0102': {'B0203': 90, 'BP0103': 0},
    'BP0103': {'BP0203': -90},
    'BP0104': {'B0205': 90, 'BP0105': 0},
    'BP0105': {'BP0205': -90},
    'BP0106': {'BP0206': -90},
    'BP0107': {'BP0207': -90},
    'BP0108': {'BP0208': -90},
    'BP0109': {'B0211': 90, 'BP0108': 180},
    'BP0110': {'BP0210': -90},
    'BP0111': {'B0213': 90, 'BP0110': 180},
    'BP0112': {'BP0212': -90},

    'BP0201': {'B0302': -90},
    'BP0202': {'BP0102': 90},
    'BP0203': {'B0304': -90, 'BP0202': 180},
    'BP0204': {'BP0104': 90},
    'BP0205': {'B0306': -90, 'BP0204': 180},
    'BP0206': {'B0307': -90},
    'BP0207': {'B0309': -90},
    'BP0208': {'B0310': -90, 'BP0209': 0},
    'BP0209': {'BP0109': 90},
    'BP0210': {'B0312': -90, 'BP0211': 0},
    'BP0211': {'BP0111': 90},
    'BP0212': {'B0314': -90},

    'B0301': {'B0201': 90, 'B0302': 0},
    'B0302': {'B0402': -90, 'B0303': 0},
    'B0303': {'BP0202': 90, 'B0304': 0},
    'B0304': {'B0404': -90, 'B0305': 0},
    'B0305': {'BP0204': 90, 'B0306': 0},
    'B0306': {'B0406': -90, 'B0307': 0},
    'B0307': {'B0407': -90},
    'B0308': {'B0208': 90},
    'B0309': {'B0409': -90},
    'B0310': {'B0410': -90, 'B0309': 180},
    'B0311': {'BP0209': 90, 'B0310': 180},
    'B0312': {'B0412': -90, 'B0311': 180},
    'B0313': {'BP0211': 90, 'B0312': 180},
    'B0314': {'B0414': -90, 'B0313': 180},
    'B0315': {'B0215': 90, 'B0314': 180},

    'B0401': {'B0301': 90},
    'B0402': {'BP0301': -90, 'B0401': 180},
    'B0403': {'B0303': 90, 'B0402': 180},
    'B0404': {'BP0303': -90, 'B0403': 180},
    'B0405': {'B0305': 90, 'B0404': 180},
    'B0406': {'BP0305': -90, 'B0405': 180},
    'B0407': {'BP0306': -90, 'B0406': 180},
    'B0408': {'B0308': 90, 'B0407': 180, 'B0409': 0},
    'B0409': {'BP0307': -90, 'B0410': 0},
    'B0410': {'BP0308': -90, 'B0411': 0},
    'B0411': {'B0311': 90, 'B0412': 0},
    'B0412': {'BP0310': -90, 'B0413': 0},
    'B0413': {'B0313': 90, 'B0414': 0},
    'B0414': {'BP0312': -90, 'B0415': 0},
    'B0415': {'B0315': 90},

    'BP0301': {'BP0401': -90},
    'BP0302': {'B0403': 90, 'BP0303': 0},
    'BP0303': {'BP0403': -90},
    'BP0304': {'B0405': 90, 'BP0305': 0},
    'BP0305': {'BP0405': -90},
    'BP0306': {'BP0406': -90},
    'BP0307': {'BP0407': -90},
    'BP0308': {'BP0408': -90},
    'BP0309': {'B0411': 90, 'BP0308': 180},
    'BP0310': {'BP0410': -90},
    'BP0311': {'B0413': 90, 'BP0310': 180},
    'BP0312': {'BP0412': -90},

    'BP0401': {'B0502': -90},
    'BP0402': {'BP0302': 90},
    'BP0403': {'B0504': -90, 'BP0402': 180},
    'BP0404': {'BP0304': 90},
    'BP0405': {'B0506': -90, 'BP0404': 180},
    'BP0406': {'B0507': -90},
    'BP0407': {'B0509': -90},
    'BP0408': {'B0510': -90, 'BP0409': 0},
    'BP0409': {'BP0309': 90},
    'BP0410': {'B0512': -90, 'BP0411': 0},
    'BP0411': {'BP0311': 90},
    'BP0412': {'B0514': -90},

    'B0501': {'B0401': 90, 'B0502': 0},
    'B0502': {'B0602': -90, 'B0503': 0},
    'B0503': {'BP0402': 90, 'B0504': 0},
    'B0504': {'B0604': -90, 'B0505': 0},
    'B0505': {'BP0404': 90, 'B0506': 0},
    'B0506': {'B0606': -90, 'B0507': 0},
    'B0507': {'B0607': -90},
    'B0508': {'B0408': 90},
    'B0509': {'B0609': -90},
    'B0510': {'B0610': -90, 'B0509': 180},
    'B0511': {'BP0409': 90, 'B0510': 180},
    'B0512': {'B0612': -90, 'B0511': 180},
    'B0513': {'BP0411': 90, 'B0512': 180},
    'B0514': {'B0614': -90, 'B0513': 180},
    'B0515': {'B0415': 90, 'B0514': 180},

    'B0601': {'B0501': 90},
    'B0602': {'BP0501': -90, 'B0601': 180},
    'B0603': {'B0503': 90, 'B0602': 180},
    'B0604': {'BP0503': -90, 'B0603': 180},
    'B0605': {'B0505': 90, 'B0604': 180},
    'B0606': {'BP0505': -90, 'B0605': 180},
    'B0607': {'BP0506': -90, 'B0606': 180},
    'B0608': {'B0508': 90, 'B0607': 180, 'B0609': 0},
    'B0609': {'BP0507': -90, 'B0610': 0},
    'B0610': {'BP0508': -90, 'B0611': 0},
    'B0611': {'B0511': 90, 'B0612': 0},
    'B0612': {'BP0510': -90, 'B0613': 0},
    'B0613': {'B0513': 90, 'B0614': 0},
    'B0614': {'BP0512': -90, 'B0615': 0},
    'B0615': {'B0515': 90},

    'BP0501': {'BP0601': -90},
    'BP0502': {'B0603': 90, 'BP0503': 0},
    'BP0503': {'BP0603': -90},
    'BP0504': {'B0605': 90, 'BP0505': 0},
    'BP0505': {'BP0605': -90},
    'BP0506': {'BP0606': -90},
    'BP0507': {'BP0607': -90},
    'BP0508': {'BP0608': -90},
    'BP0509': {'B0611': 90, 'BP0508': 180},
    'BP0510': {'BP0610': -90},
    'BP0511': {'B0613': 90, 'BP0510': 180},
    'BP0512': {'BP0612': -90},

    'BP0601': {'B0702': -90},
    'BP0602': {'BP0502': 90},
    'BP0603': {'B0704': -90, 'BP0602': 180},
    'BP0604': {'BP0504': 90},
    'BP0605': {'B0706': -90, 'BP0604': 180},
    'BP0606': {'B0707': -90},
    'BP0607': {'B0709': -90},
    'BP0608': {'B0710': -90, 'BP0609': 0},
    'BP0609': {'BP0509': 90},
    'BP0610': {'B0712': -90, 'BP0611': 0},
    'BP0611': {'BP0511': 90},
    'BP0612': {'B0714': -90},

    'B0701': {'B0601': 90, 'B0702': 0},
    'B0702': {'B0802': -90, 'B0703': 0},
    'B0703': {'BP0602': 90, 'B0704': 0},
    'B0704': {'B0804': -90, 'B0705': 0},
    'B0705': {'BP0604': 90, 'B0706': 0},
    'B0706': {'B0806': -90, 'B0707': 0},
    'B0707': {'B0807': -90},
    'B0708': {'B0608': 90},
    'B0709': {'B0809': -90},
    'B0710': {'B0810': -90, 'B0709': 180},
    'B0711': {'BP0609': 90, 'B0710': 180},
    'B0712': {'B0812': -90, 'B0711': 180},
    'B0713': {'BP0611': 90, 'B0712': 180},
    'B0714': {'B0814': -90, 'B0713': 180},
    'B0715': {'B0615': 90, 'B0714': 180},

    'B0801': {'B0701': 90},
    'B0802': {'B0902': -90, 'B0801': 180},
    'B0803': {'B0703': 90, 'B0802': 180},
    'B0804': {'B0904': -90, 'B0803': 180},
    'B0805': {'B0705': 90, 'B0804': 180},
    'B0806': {'B0906': -90, 'B0805': 180},
    'B0807': {'B0907': -90, 'B0806': 180},
    'B0808': {'B0708': 90, 'B0807': 180, 'B0809': 0},
    'B0809': {'B0909': -90, 'B0810': 0},
    'B0810': {'B0910': -90, 'B0811': 0},
    'B0811': {'B0711': 90, 'B0812': 0},
    'B0812': {'B0912': -90, 'B0813': 0},
    'B0813': {'B0713': 90, 'B0814': 0},
    'B0814': {'B0914': -90, 'B0815': 0},
    'B0815': {'B0715': 90},

    'B0901': {'B0801': 90, 'B0902': 0},
    'B0902': {'BO0102': -90, 'B0903': 0},
    'B0903': {'B0803': 90, 'B0904': 0},
    'B0904': {'BO0104': -90, 'B0905': 0},
    'B0905': {'B0805': 90, 'B0906': 0},
    'B0906': {'BO0106': -90, 'B0907': 0},
    'B0907': {'BO0107': -90},
    'B0908': {'B0808': 90},
    'B0909': {'BO0109': -90},
    'B0910': {'BO0110': -90, 'B0909': 180},
    'B0911': {'B0811': 90, 'B0910': 180},
    'B0912': {'BO0112': -90, 'B0911': 180},
    'B0913': {'B0813': 90, 'B0912': 180},
    'B0914': {'BO0114': -90, 'B0913': 180},
    'B0915': {'B0815': 90, 'B0914': 180},

    'BO0101': {'B0901': 90},
    'BO0102': {'BO0101': 180, 'BO0103': 0, 'B0902': 90},
    'BO0103': {'BO0102': 180, 'BO0104': 0, 'B0903': 90},
    'BO0104': {'BO0103': 180, 'BO0105': 0, 'B0904': 90},
    'BO0105': {'BO0104': 180, 'BO0106': 0, 'B0905': 90},
    'BO0106': {'BO0105': 180, 'BO0107': 0, 'B0906': 90},
    'BO0107': {'BO0106': 180, 'BO0108': 0, 'B0907': 90},
    'BO0108': {'B0908': 90},
    'BO0109': {'BO0108': 180, 'BO0110': 0, 'B0909': 90},
    'BO0110': {'BO0109': 180, 'BO0111': 0, 'B0910': 90},
    'BO0111': {'BO0110': 180, 'BO0112': 0, 'B0911': 90},
    'BO0112': {'BO0111': 180, 'BO0113': 0, 'B0912': 90},
    'BO0113': {'BO0112': 180, 'BO0114': 0, 'B0913': 90},
    'BO0114': {'BO0113': 180, 'BO0115': 0, 'B0914': 90},
    'BO0115': {'B0915': 90},

}

node = {

    'BS0101':	[0,	-10],
    'BS0102':	[0,	-9],
    'BS0103':	[0,	-7],
    'BS0104':	[0,	-6],
    'BS0105':	[0,	-4],
    'BS0106':	[0,	-3],
    'BS0107':	[0,	-1],
    'BS0108':	[0,	0],
    'BS0109':	[0,	1],
    'BS0110':	[0,	3],
    'BS0111':	[0,	4],
    'BS0112':	[0,	6],
    'BS0113':	[0,	7],
    'BS0114':	[0,	9],
    'BS0115':	[0,	10],

    'B0101': [0.5,	-10],
    'B0102': [0.5,	-9],
    'B0103': [0.5,	-7],
    'B0104': [0.5,	-6],
    'B0105': [0.5,	-4],
    'B0106': [0.5,	-3],
    'B0107': [0.5,	-1],
    'B0108': [0.5,	0],
    'B0109': [0.5,	1],
    'B0110': [0.5,	3],
    'B0111': [0.5,	4],
    'B0112': [0.5,	6],
    'B0113': [0.5,	7],
    'B0114': [0.5,	9],
    'B0115': [0.5,	10],

    'B0201': [1.5,	-10],
    'B0202': [1.5,	-9],
    'B0203': [1.5,	-7],
    'B0204': [1.5,	-6],
    'B0205': [1.5,	-4],
    'B0206': [1.5,	-3],
    'B0207': [1.5,	-1],
    'B0208': [1.5,	0],
    'B0209': [1.5,	1],
    'B0210': [1.5,	3],
    'B0211': [1.5,	4],
    'B0212': [1.5,	6],
    'B0213': [1.5,	7],
    'B0214': [1.5,	9],
    'B0215': [1.5,	10],

    'BP0101': [2.5,	-9],
    'BP0102': [2.5,	-7],
    'BP0103': [2.5,	-6],
    'BP0104': [2.5,	-4],
    'BP0105': [2.5,	-3],
    'BP0106': [2.5,	-1],
    'BP0107': [2.5,	1],
    'BP0108': [2.5,	3],
    'BP0109': [2.5,	4],
    'BP0110': [2.5,	6],
    'BP0111': [2.5,	7],
    'BP0112': [2.5,	9],

    'BP0201':	[3.5,	-9],
    'BP0202':	[3.5,	-7],
    'BP0203':	[3.5,	-6],
    'BP0204':	[3.5,	-4],
    'BP0205':	[3.5,	-3],
    'BP0206':	[3.5,	-1],
    'BP0207':	[3.5,	1],
    'BP0208':	[3.5,	3],
    'BP0209':	[3.5,	4],
    'BP0210':	[3.5,	6],
    'BP0211':	[3.5,	7],
    'BP0212':	[3.5,	9],

    'B0301': [4.5,	-10],
    'B0302': [4.5,	-9],
    'B0303': [4.5,	-7],
    'B0304': [4.5,	-6],
    'B0305': [4.5,	-4],
    'B0306': [4.5,	-3],
    'B0307': [4.5,	-1],
    'B0308': [4.5,	0],
    'B0309': [4.5,	1],
    'B0310': [4.5,	3],
    'B0311': [4.5,	4],
    'B0312': [4.5,	6],
    'B0313': [4.5,	7],
    'B0314': [4.5,	9],
    'B0315': [4.5,	10],

    'B0401': [5.5,	-10],
    'B0402': [5.5,	-9],
    'B0403': [5.5,	-7],
    'B0404': [5.5,	-6],
    'B0405': [5.5,	-4],
    'B0406': [5.5,	-3],
    'B0407': [5.5,	-1],
    'B0408': [5.5,	0],
    'B0409': [5.5,	1],
    'B0410': [5.5,	3],
    'B0411': [5.5,	4],
    'B0412': [5.5,	6],
    'B0413': [5.5,	7],
    'B0414': [5.5,	9],
    'B0415': [5.5,	10],

    'BP0301': [6.5,	-9],
    'BP0302': [6.5,	-7],
    'BP0303': [6.5,	-6],
    'BP0304': [6.5,	-4],
    'BP0305': [6.5,	-3],
    'BP0306': [6.5,	-1],
    'BP0307': [6.5,	1],
    'BP0308': [6.5,	3],
    'BP0309': [6.5,	4],
    'BP0310': [6.5,	6],
    'BP0311': [6.5,	7],
    'BP0312': [6.5,	9],

    'BP0401': [7.5,	-9],
    'BP0402': [7.5,	-7],
    'BP0403': [7.5,	-6],
    'BP0404': [7.5,	-4],
    'BP0405': [7.5,	-3],
    'BP0406': [7.5,	-1],
    'BP0407': [7.5,	1],
    'BP0408': [7.5,	3],
    'BP0409': [7.5,	4],
    'BP0410': [7.5,	6],
    'BP0411': [7.5,	7],
    'BP0412': [7.5,	9],

    'B0501': [8.5,-10],
    'B0502': [8.5,-9],
    'B0503': [8.5,-7],
    'B0504': [8.5,-6],
    'B0505': [8.5,-4],
    'B0506': [8.5,-3],
    'B0507': [8.5,-1],
    'B0508': [8.5,0],
    'B0509': [8.5,1],
    'B0510': [8.5,3],
    'B0511': [8.5,4],
    'B0512': [8.5,6],
    'B0513': [8.5,7],
    'B0514': [8.5,9],
    'B0515': [8.5,10],

    'B0601': [9.5,	-10],
    'B0602': [9.5,	-9],
    'B0603': [9.5,	-7],
    'B0604': [9.5,	-6],
    'B0605': [9.5,	-4],
    'B0606': [9.5,	-3],
    'B0607': [9.5,	-1],
    'B0608': [9.5,	0],
    'B0609': [9.5,	1],
    'B0610': [9.5,	3],
    'B0611': [9.5,	4],
    'B0612': [9.5,	6],
    'B0613': [9.5,	7],
    'B0614': [9.5,	9],
    'B0615': [9.5,	10],

    'BP0501': [10.5,	-9],
    'BP0502': [10.5,	-7],
    'BP0503': [10.5,	-6],
    'BP0504': [10.5,	-4],
    'BP0505': [10.5,	-3],
    'BP0506': [10.5,	-1],
    'BP0507': [10.5,	1],
    'BP0508': [10.5,	3],
    'BP0509': [10.5,	4],
    'BP0510': [10.5,	6],
    'BP0512': [10.5,	9],
    'BP0511': [10.5,	7],

    'BP0601': [11.5,	-9],
    'BP0602': [11.5,	-7],
    'BP0603': [11.5,	-6],
    'BP0604': [11.5,	-4],
    'BP0605': [11.5,	-3],
    'BP0606': [11.5,	-1],
    'BP0607': [11.5,	1],
    'BP0608': [11.5,	3],
    'BP0609': [11.5,	4],
    'BP0610': [11.5,	6],
    'BP0611': [11.5,	7],
    'BP0612': [11.5,	9],

    'B0701': [12.5,	-10],
    'B0702': [12.5,	-9],
    'B0703': [12.5,	-7],
    'B0704': [12.5,	-6],
    'B0705': [12.5,	-4],
    'B0706': [12.5,	-3],
    'B0707': [12.5,	-1],
    'B0708': [12.5,	0],
    'B0709': [12.5,	1],
    'B0710': [12.5,	3],
    'B0711': [12.5,	4],
    'B0712': [12.5,	6],
    'B0713': [12.5,	7],
    'B0714': [12.5,	9],
    'B0715': [12.5,	10],

    'B0801': [13.5,	-10],
    'B0802': [13.5,	-9],
    'B0803': [13.5,	-7],
    'B0804': [13.5,	-6],
    'B0805': [13.5,	-4],
    'B0806': [13.5,	-3],
    'B0807': [13.5,	-1],
    'B0808': [13.5,	0],
    'B0809': [13.5,	1],
    'B0810': [13.5,	3],
    'B0811': [13.5,	4],
    'B0812': [13.5,	6],
    'B0813': [13.5,	7],
    'B0814': [13.5,	9],
    'B0815': [13.5,	10],

    'B0901': [14.5,	-10],
    'B0902': [14.5,	-9],
    'B0903': [14.5,	-7],
    'B0904': [14.5,	-6],
    'B0905': [14.5,	-4],
    'B0906': [14.5,	-3],
    'B0907': [14.5,	-1],
    'B0908': [14.5,	0],
    'B0909': [14.5,	1],
    'B0910': [14.5,	3],
    'B0911': [14.5,	4],
    'B0912': [14.5,	6],
    'B0913': [14.5,	7],
    'B0914': [14.5,	9],
    'B0915': [14.5,	10],

    'BO0101': [15,	-10],
    'BO0102': [15,	-9],
    'BO0103': [15,	-7],
    'BO0104': [15,	-6],
    'BO0105': [15,	-4],
    'BO0106': [15,	-3],
    'BO0107': [15,	-1],
    'BO0108': [15,	0],
    'BO0109': [15,	1],
    'BO0110': [15,	3],
    'BO0111': [15,	4],
    'BO0112': [15,	6],
    'BO0113': [15,	7],
    'BO0114': [15,	9],
    'BO0115': [15,	10]

}



def callback(msg):
    try:
        shortest_dist = addCoordinates(dijkstra(graph, msg.startNode, msg.endNode))
        loc = Locations()
        for i in range(len(shortest_dist)):
            coord = Coordinate()
            coord.QR = shortest_dist[i][0]
            coord.x = shortest_dist[i][2][1]
            coord.y = -shortest_dist[i][2][0]
            coord.deg = shortest_dist[i][1]
            loc.location.append(coord)
        pub.publish(loc)
    except:
        print('error')
    

if __name__ == '__main__':
    rospy.init_node('path_pub')
    pub = rospy.Publisher('path', Locations, queue_size=1)
    sub = rospy.Subscriber('move', Move, callback)
    rospy.spin()