classdef DigraphTree
% Manipulate a digraph in form of a search tree: each child has exactly one
% parent
    
    methods(Static)
        function dG = initialise(rootNode, color)
        % Initialise the tree with one root node
            
            dG = digraph();
            nodeProperties = table({rootNode}, {color}, 'VariableNames', {'Name', 'Color'});
            dG = addnode(dG, nodeProperties);
        end
        
        function [dG, child, childID] = expand(dG, parent, state, edgeName, colorNode, colorEdge, safety)
        % Expand the tree, add a new state node and connect it to its parent
            
            childID = DigraphTree.getNewID();
            
            child = DigraphTree.getNodeName(childID, state, safety);
            dG = DigraphTree.connect(dG, parent, child, edgeName, colorNode, colorEdge);
        end
        
        function dG = connect(dG, parent, child, edgeName, colorNode, colorEdge)
        % Add a new child and connect it with its parent 
        
            nodeProperties = table({child}, {colorNode}, 'VariableNames', {'Name', 'Color'});
            dG = addnode(dG, nodeProperties);

            edgeProperties = table({edgeName}, {colorEdge}, 'VariableNames', {'Power', 'Color'});
            dG = addedge(dG, parent, child, edgeProperties);
        end
        
        function dG = changeNodeColor(dG, node, color)
        % Change the color of an existing node
            
            id_node = findnode(dG, node);
            dG.Nodes.Color{id_node} = color;
        end
        
        function dG = changeEdgeColor(dG, parent, child, color)
        % Change the color of an existing unique edge
        
            id_Edge = findedge(dG, parent, child);
            dG.Edges.Color{id_Edge} = color;
        end
        
        function nodeName = getNodeName(ID, state, safety)
        % Get unique node name defined by its ID
            
            nodeName = ['ID:', dec2hex(ID), ...
                        ' s:', '[', regexprep(num2str(round([state.s], 1)), '\s+',' '), ']', ...
                        ', d:', '[', regexprep(num2str(round([state.d], 1)), '\s+',' '), ']', ...
                        ', v:', '[', regexprep(num2str(round([state.speed], 1)), '\s+',' '), ']', ...
                        ', S_{f}:', num2str(round(safety, 1))];
        end
        
        function ID_global = getNewID(init)
        % Get new unique IDs

            persistent ID;
            if nargin == 1
                ID = init;
            else
                ID = ID + 1;
            end
            ID_global = ID;
        end
    end
end

