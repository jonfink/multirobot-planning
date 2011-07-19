load nodes.txt
load edges.txt
load optpath.txt

figure
daspect ([1 1 1]);


% Draw the nodes
max_a = max(nodes(:,1));
hold on, plot (nodes (:,1), nodes(:,2), 'g.')

% (1 2 3 4) (5 6 7 8)
% Draw the edges 
for i = 1 : length (edges) 
      hold on, plot ( [edges(i,1), edges(i,3)], [edges(i,2), edges(i,4)], 'Color', [0.1 0.5 0.7], 'LineWidth', 0.5);
end

plot(optpath(:,1), optpath(:,2), 'r-', 'LineWidth', 4.0)
