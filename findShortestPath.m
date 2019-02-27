function [path_edges,distance_min] = findShortestPath...
    (G,node_table,pStart,eStart,pEnd,eEnd,road_network)
%% Input:
% G: graph for further earch
% node_table: correspoding relationship between NodeID and GraphID
% pStart,pEnd: starting points and ending points [lon lat]
% eStart,eEnd: correspoding candidate edges of pStart,pEnd
eStart = road_network(road_network.EdgeID == eStart,:);
eEnd = road_network(road_network.EdgeID == eEnd,:);

%%
if eStart.EdgeID == eEnd.EdgeID
    distance_min = deg2km(distance(fliplr(pStart),fliplr(pEnd)));
    path_edges = eStart.EdgeID;
    p_start = pStart;p_end = pEnd;
else
    Node1ofStart = eStart.Node1ID;Node2ofStart = eStart.Node2ID;
    Node1ofEnd = eEnd.Node1ID;Node2ofEnd = eEnd.Node2ID;
    % direction vectors
    vecPoints = pEnd-pStart;
    vecEdge1 = [eStart.Node2Lon eStart.Node2Lat] - [eStart.Node1Lon eStart.Node1Lat];
    vecEdge2 = [eEnd.Node2Lon eEnd.Node2Lat] - [eEnd.Node1Lon eEnd.Node1Lat];
    cross2 = @(vec1,vec2) sum(vec1.*vec2); % cross product for 2-d vector
    % comparing direction vectors to determine head and tail node of
    % each edge.
    % directly adding node in graph may be harmful and logically incorrect
    if all(pStart==Node1ofStart)
        eStart_tail = Node1ofStart;
        p_start = [eStart.Node1Lon eStart.Node1Lat];
    elseif all(pStart==Node2ofStart)
        eStart_tail = Node2ofStart;
        p_start = [eStart.Node2Lon eStart.Node2Lat];
    elseif cross2(vecPoints,vecEdge1) >= 0
        eStart_tail = Node2ofStart;
        p_start = [eStart.Node2Lon eStart.Node2Lat];
    else
        eStart_tail = Node1ofStart;
        p_start = [eStart.Node1Lon eStart.Node1Lat];
    end
    if all(pEnd == Node1ofEnd)
        eEnd_head = Node1ofEnd;
        p_end = [eEnd.Node1Lon eEnd.Node1Lat];
    elseif all(pEnd == Node2ofEnd)
        eEnd_head = Node2ofEnd;
        p_end = [eEnd.Node2Lon eEnd.Node2Lat];
    elseif cross2(vecPoints,vecEdge2) >= 0
        eEnd_head = Node1ofEnd;
        p_end = [eEnd.Node1Lon eEnd.Node1Lat];
    else
        eEnd_head = Node2ofEnd;
        p_end = [eEnd.Node2Lon eEnd.Node2Lat];
    end
    if all(p_start == p_end)
        distance_min = deg2km(distance(fliplr(pStart),fliplr(p_start))) ...
            + deg2km(distance(fliplr(pEnd),fliplr(p_end)));
        path_edges = [eStart.EdgeID,eEnd.EdgeID];
    else
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
            ppStart = P(p_idx-1);ppEnd = P(p_idx);
            ppStart = node_table.nodeID(node_table.nodeGraph == ppStart);
            ppEnd = node_table.nodeID(node_table.nodeGraph == ppEnd);
            edge_res_1 = road_network.EdgeID(road_network.Node1ID == ppStart ...
                & road_network.Node2ID == ppEnd);
            edge_res_2 = road_network.EdgeID(road_network.Node1ID == ppEnd ...
                & road_network.Node2ID == ppStart);
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
end
h1 = plot(pStart(1),pStart(2),'ro');h2 = plot(pEnd(1),pEnd(2),'bo');
h3 =  plot(p_start(1),p_start(2),'rx');h4 = plot(p_end(1),p_end(2),'bx');
path_edges_raw =  road_network(ismember(road_network.EdgeID,path_edges),:);
h5 = plot([path_edges_raw.Node1Lon,path_edges_raw.Node2Lon]',[path_edges_raw.Node1Lat,path_edges_raw.Node2Lat]','kd');
h6 = plot([eStart.Node1Lon,eStart.Node2Lon],[eStart.Node1Lat,eStart.Node2Lat],'rd');
h7 = plot([eEnd.Node1Lon,eEnd.Node2Lon],[eEnd.Node1Lat,eEnd.Node2Lat],'bd');
delete(h1);delete(h2);delete(h3);delete(h4);delete(h5);delete(h6);delete(h7);
plot([path_edges_raw.Node1Lon,path_edges_raw.Node2Lon]',[path_edges_raw.Node1Lat,path_edges_raw.Node2Lat]','k-')
end