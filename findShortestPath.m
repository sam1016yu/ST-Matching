function [path_edges,distance_min] = findShortestPath...
    (G,node_table,pStart,eStart,pEnd,eEnd,road_network)
   %% Input:
    % G: graph for further earch
    % node_table: correspoding relationship between NodeID and GraphID
    % pStart,pEnd: starting points and ending points [lon lat]
    % eStart,eEnd: correspoding candidate edges of pStart,pEnd
    eStart = road_network(road_network.EdgeID == eStart,:);
    eEnd = road_network(road_network.EdgeID == eEnd,:);
    Node1Start = eStart.Node1ID;Node2Start = eStart.Node2ID;
    Node1End = eEnd.Node1ID;Node2End = eEnd.Node2ID;
    %%
    % direction vectors
    vecPoints = pEnd-pStart;
    vecEdge1 = [eStart.Node2Lon eStart.Node2Lat] - [eStart.Node1Lon eStart.Node1Lat];
    vecEdge2 = [eEnd.Node2Lon eEnd.Node2Lat] - [eEnd.Node1Lon eEnd.Node1Lat];
    cross2 = @(vec1,vec2) sum(vec1.*vec2); % cross product for 2-d vector
    
    if cross2(vecPoints,vecEdge1) >= 0
        eStart_tail = Node2Start;
        p_start = [eStart.Node2Lon eStart.Node2Lat];
    else
        eStart_tail = Node1Start;
        p_start = [eStart.Node1Lon eStart.Node1Lat];
    end
    if cross2(vecPoints,vecEdge2) >= 0
        eEnd_head = Node1End;
        p_end = [eEnd.Node1Lon eEnd.Node1Lat];
    else
        eEnd_head = Node2End;
        p_end = [eEnd.Node2Lon eEnd.Node2Lat];
    end
    %%
    graphNode_start = node_table.nodeGraph(node_table.nodeID == eStart_tail);
    graphNode_end = node_table.nodeGraph(node_table.nodeID == eEnd_head);
    % shortest distance is found between the tail node of start edge and
    % head node of end edge
    [P,d] = shortestpath(G,graphNode_start,graphNode_end);
    distance_min = d + ...
        distance(fliplr(pStart),fliplr(p_start),almanac('earth', 'wgs84')) ...
        + distance(fliplr(pEnd),fliplr(p_end), almanac('earth', 'wgs84'));
    %%
    
    P_edges = zeros(1,length(P)-1);
    
    for p_idx = 2:length(P)
        edge_res_1 = [];edge_res_2 = [];
        pStart = P(p_idx-1);pEnd = P(p_idx);
        pStart = node_table.nodeID(node_table.nodeGraph == pStart);
        pEnd = node_table.nodeID(node_table.nodeGraph == pEnd);
        edge_res_1 = road_network.EdgeID(road_network.Node1ID == pStart ...
            & road_network.Node2ID == pEnd);
        edge_res_2 = road_network.EdgeID(road_network.Node1ID == pEnd ...
            & road_network.Node2ID == pStart);
        P_edges(p_idx-1) = [edge_res_1 edge_res_2];
    end
    path_edges = [eStart.EdgeID P_edges eEnd.EdgeID];
end