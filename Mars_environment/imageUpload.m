clf;
% top left, top right ; bottom right, bottom left
surf([-8,-8;8,8],[-8,8;-8,8],[0,0;0,0],'CData',imread('ground_mars.jpg'),'FaceColor','texturemap');
surf([8,-8;8,-8],[8,8;8,8],[5,5;0,0],'CData',imread('wall_mars.jpg'),'FaceColor','texturemap');

surf([8,8;8,8],[8,-8;8,-8],[5,5;0,0],'CData',imread('wall_mars_1.jpg'),'FaceColor','texturemap');
axis on;


