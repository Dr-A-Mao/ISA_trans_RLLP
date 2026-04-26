%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% drawMap
% plot obstacles and path
function drawMap(map)%,path,smoothedPath,tree,R_min)
  % draw buildings 
  V = [];
  F = [];
  patchcolors = [];
  count = 0;
  for i=1:length(map.buildings_n)
      for j=1:length(map.buildings_e)
        [Vtemp,Ftemp,patchcolorstemp] = buildingVertFace(map.buildings_n(i),...
            map.buildings_e(j),map.BuildingWidth,map.heights(j,i));
        V = [V; Vtemp];
        Ftemp = Ftemp + count;
        F = [F; Ftemp];
        count = count + 8;
        patchcolors = [patchcolors;patchcolorstemp];
      end
  end
  
  patch('Vertices', V, 'Faces', F,...
        'FaceVertexCData',patchcolors,...
        'FaceColor','flat','DisplayName','Buildings');
 

end