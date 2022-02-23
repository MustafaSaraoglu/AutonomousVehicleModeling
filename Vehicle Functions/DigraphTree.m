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
        
        function [dG, child, ID] = expand(dG, ID, parent, state, edgeName, colorNode, colorEdge, unsafety)
        % Expand the tree, add a new state node and connect it to its parent
            
            ID = ID + 1;
            
            child = DigraphTree.getNodeName(ID, state, unsafety);
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
        
        function nodeName = getNodeName(ID, state, unsafety)
        % Get unique node name defined by its ID
            
            nodeName = ['ID:', dec2hex(ID), ...
                        ' s:', '[', regexprep(num2str(round([state.s], 1)), '\s+',' '), ']', ...
                        ', d:', '[', regexprep(num2str(round([state.d], 1)), '\s+',' '), ']', ...
                        ', v:', '[', regexprep(num2str(round([state.speed], 1)), '\s+',' '), ']', ...
                        ', U:', num2str(round(unsafety, 1))];
        end
    end
end

