function [path_edges,distance_min] = findShortestPath...
    (G,node_table,pStart,eStart,pEnd,eEnd,road_network)
%% Input:
% G: graph for further earch
% node_table: correspoding relationship between NodeID and GraphID
% pStart,pEnd: starting points and ending points [lon lat]
% eStart,eEnd: correspoding candidate edges of pStart,pEnd
road_network = road_network{:,1:7};
eStart = road_network(road_network(:,1) == eStart,:);
eEnd = road_network(road_network(:,1) == eEnd,:);

%%
if eStart(:,1) == eEnd(:,1)
    distance_min = deg2km(distance(fliplr(pStart),fliplr(pEnd)));
    path_edges = eStart(:,1);
%     p_start = pStart;p_end = pEnd;
else
    Node1ofStart = eStart(:,2);Node2ofStart = eStart(:,3);
    Node1ofEnd = eEnd(:,2);Node2ofEnd = eEnd(:,3);
    % direction vectors
    vecPoints = pEnd-pStart;
    vecEdge1 = [eStart(:,6) eStart(:,7)] - [eStart(:,4) eStart(:,5)];
    vecEdge2 = [eEnd(:,6) eEnd(:,7)] - [eEnd(:,4) eEnd(:,5)];
    cross2 = @(vec1,vec2) sum(vec1.*vec2); % cross product for 2-d vector
    % comparing direction vectors to determine head and tail node of
    % each edge.
    % directly adding node in graph may be harmful and logically incorrect
    if all(pStart==Node1ofStart)
        eStart_tail = Node1ofStart;
        p_start = [eStart(:,4) eStart(:,5)];
    elseif all(pStart==Node2ofStart)
        eStart_tail = Node2ofStart;
        p_start = [eStart(:,6) eStart(:,7)];
    elseif cross2(vecPoints,vecEdge1) >= 0
        eStart_tail = Node2ofStart;
        p_start = [eStart(:,6) eStart(:,7)];
    else
        eStart_tail = Node1ofStart;
        p_start = [eStart(:,4) eStart(:,5)];
    end
    if all(pEnd == Node1ofEnd)
        eEnd_head = Node1ofEnd;
        p_end = [eEnd(:,4) eEnd(:,5)];
    elseif all(pEnd == Node2ofEnd)
        eEnd_head = Node2ofEnd;
        p_end = [eEnd(:,6) eEnd(:,7)];
    elseif cross2(vecPoints,vecEdge2) >= 0
        eEnd_head = Node1ofEnd;
        p_end = [eEnd(:,4) eEnd(:,5)];
    else
        eEnd_head = Node2ofEnd;
        p_end = [eEnd(:,6) eEnd(:,7)];
    end
    if all(p_start == p_end)
        distance_min = deg2km(distance(fliplr(pStart),fliplr(p_start))) ...
            + deg2km(distance(fliplr(pEnd),fliplr(p_end)));
        path_edges = [eStart(:,1),eEnd(:,1)];
    else
        %% starting and ending node of path search
        graphNode_start = node_table.nodeGraph(node_table.nodeID == eStart_tail);
        graphNode_end = node_table.nodeGraph(node_table.nodeID == eEnd_head);
        % shortest distance is found between the tail node of start edge and
        % head node of end edge
        [P,d] = shortestpath(G,graphNode_start,graphNode_end);
        start2tail = deg2km(distance(fliplr(pStart),fliplr(p_start)));
        end2head =  deg2km(distance(fliplr(pEnd),fliplr(p_end)));
        % total distance calculation
        distance_min = d + start2tail + end2head;
        %% adding edges passed according to the path found
        P_edges = zeros(1,length(P)-1);
        for p_idx = 2:length(P)
            ppStart = P(p_idx-1);ppEnd = P(p_idx);
            ppStart = node_table.nodeID(node_table.nodeGraph == ppStart);
            ppEnd = node_table.nodeID(node_table.nodeGraph == ppEnd);
            if any(road_network(:,2) == ppStart & road_network(:,3) == ppEnd)
                edg_loc = (road_network(:,2) == ppStart & road_network(:,3) == ppEnd);
                matched_edge = road_network(edg_loc,1);
            else
                edg_loc = (road_network(:,2) == ppEnd  & road_network(:,3) == ppStart);
                matched_edge = road_network(edg_loc,1);
            end
            % if more than one edge is found, return only one
            P_edges(p_idx-1) = matched_edge(1);
        end
        if any(P_edges == eStart(:,1))
            distance_min = distance_min - 2*start2tail;
        end
        if any(P_edges == eEnd(:,1))
            distance_min = distance_min - 2*end2head;
        end
        path_edges = unique([eStart(:,1) P_edges eEnd(:,1)]);
        
        % d = Inf means no path!
        % not sure of the reason yet
%         assert(d<Inf, 'shortestPath:noPath','Could not find a path between candidate points!');
    end
end
% h1 = plot(pStart(1),pStart(2),'ro');h2 = plot(pEnd(1),pEnd(2),'bo');
% h3 =  plot(p_start(1),p_start(2),'rx');h4 = plot(p_end(1),p_end(2),'bx');
% path_edges_raw =  road_network(ismember(road_network(:,1),path_edges),:);
% h5 = plot([path_edges_raw(:,4),path_edges_raw(:,6)]',[path_edges_raw(:,5),path_edges_raw(:,7)]','kd');
% h6 = plot([eStart(:,4),eStart(:,6)],[eStart(:,5),eStart(:,7)],'rd');
% h7 = plot([eEnd(:,4),eEnd(:,6)],[eEnd(:,5),eEnd(:,7)],'bd');
% delete(h1);delete(h2);delete(h3);delete(h4);delete(h5);delete(h6);delete(h7);
% plot([path_edges_raw(:,4),path_edges_raw(:,6)]',[path_edges_raw(:,5),path_edges_raw(:,7)]','k-')
end