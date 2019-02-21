function [path_edges,distance_min] = findShortestPath...
    (G,node_table,pStart,eStart,pEnd,eEnd,road_network)
   %% Input:
    % G: graph for further earch
    % node_table: correspoding relationship between NodeID and GraphID
    % pStart,pEnd: starting points and ending points [lon lat]
    % eStart,eEnd: correspoding candidate edges of pStart,pEnd
    eStart = road_network(road_network.EdgeID == eStart,:);
    eEnd = road_network(road_network.EdgeID == eEnd,:);
    Node1ofStart = eStart.Node1ID;Node2ofStart = eStart.Node2ID;
    Node1ofEnd = eEnd.Node1ID;Node2ofEnd = eEnd.Node2ID;
    %%
    % direction vectors
    vecPoints = pEnd-pStart;
    vecEdge1 = [eStart.Node2Lon eStart.Node2Lat] - [eStart.Node1Lon eStart.Node1Lat];
    vecEdge2 = [eEnd.Node2Lon eEnd.Node2Lat] - [eEnd.Node1Lon eEnd.Node1Lat];
    cross2 = @(vec1,vec2) sum(vec1.*vec2); % cross product for 2-d vector
    % comparing direction vectors to determine head and tail node of 
    % each edge.
    % directly adding node in graph may be harmful and logically incorrect
    if cross2(vecPoints,vecEdge1) >= 0
        eStart_tail = Node2ofStart;
        p_start = [eStart.Node2Lon eStart.Node2Lat];
    else
        eStart_tail = Node1ofStart;
        p_start = [eStart.Node1Lon eStart.Node1Lat];
    end
    if cross2(vecPoints,vecEdge2) >= 0
        eEnd_head = Node1ofEnd;
        p_end = [eEnd.Node1Lon eEnd.Node1Lat];
    else
        eEnd_head = Node2ofEnd;
        p_end = [eEnd.Node2Lon eEnd.Node2Lat];
    end
    %% starting and ending node of path search
    graphNode_start = node_table.nodeGraph(node_table.nodeID == eStart_tail);
    graphNode_end = node_table.nodeGraph(node_table.nodeID == eEnd_head);
    % shortest distance is found between the tail node of start edge and
    % head node of end edge
    [P,d] = shortestpath(G,graphNode_start,graphNode_end);
    % total distance calculation
    distance_min = d + ...
        deg2km(distance(fliplr(pStart),fliplr(p_start))) ...
        + deg2km(distance(fliplr(pEnd),fliplr(p_end)));
    %% adding edges passed according to the path found
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
        matched_edge = [edge_res_1 edge_res_2];
        % if more than one edge if found, return only one
        if length(matched_edge > 1)
            matched_edge = matched_edge(1);
        end
        P_edges(p_idx-1) = matched_edge;
    end
    path_edges = [eStart.EdgeID P_edges eEnd.EdgeID];
    path_edges = unique(path_edges);
    % d = Inf means no path!
    % not sure of the reason yet
    assert(d<Inf, 'shortestPath:noPath','Could not find a path between candidate points!');
end