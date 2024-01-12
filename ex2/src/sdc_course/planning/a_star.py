from .graph import Graph
import math


def heuristic(node, goal):
    """
    A heuristic function to estimate the cost from node1 to node2.
    In this example, it uses Euclidean distance.
    """
    x1, y1 = node
    x2, y2 = goal
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def a_star_search(graph: Graph, node_to_xy, start: int, end: int):
    """
    Returns shortest path as a list of nodes ids.

    : param graph : graph definition.
    : param node_to_xy : mapping of nodes to x and y positions.
    : param start : id of the start node.
    : param end : id of the end/target node.
    """
    open_list = [start]  # List of nodes that have not been visited yet
    closed_list = []  # List of nodes that have been visited
    came_from = {start: None}  # Keep track of best predecessor of each node
    accumulated_cost = {start: 0}  # Keep track of each node and its accumulated cost g
    estimated_cost = {start: 0}  # Keep track of each node and its estimated cost f=g+h
    #######################################################################
    ######################### TODO: IMPLEMENT THIS ########################
    estimated_cost[start] = accumulated_cost[start] + heuristic(node_to_xy[start], node_to_xy[end])    
    while(open_list):
        u = min(open_list, key=lambda node: estimated_cost[node])
        open_list.remove(u)
        closed_list.append(u)

        u_accumulated_cost = accumulated_cost[u]

        if u == end:
            path = []
            while u is not None:
                # Inserting always at position 0 because we are going backward now, from goal node to start node, along shortest path
                # So in the end, node start will be at position 0
                path.insert(0, u)
                u = came_from[u]
            return path

        for v in graph.get_children(u):
            if v in closed_list:
                continue

            uv_edge_cost = graph.get_cost(u, v)
            
            # If v is already in the list and was waiting to be selected, try calculating new scores based on this other reach to node v
            if v in open_list:
                # If this path to this node v is better then previous one, replace it with this new one
                if((u_accumulated_cost + uv_edge_cost + heuristic(node_to_xy[v], node_to_xy[end])) < estimated_cost[v]):
                    estimated_cost[v] = (u_accumulated_cost + uv_edge_cost + heuristic(node_to_xy[v], node_to_xy[end]))
                    accumulated_cost[v] = (u_accumulated_cost + uv_edge_cost)
                    came_from[v] = u
            
            # If v is not in the list, put it to the list
            else:
                open_list.append(v)
                estimated_cost[v] = (u_accumulated_cost + uv_edge_cost + heuristic(node_to_xy[v], node_to_xy[end]))
                accumulated_cost[v] = (u_accumulated_cost + uv_edge_cost)
                came_from[v] = u
        

    #######################################################################
    # Path to goald was not found
    return []