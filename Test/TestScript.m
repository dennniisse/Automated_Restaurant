%%
clf; clc; clear all;
L(1) = Link([pi    0      0   pi/2    1   0]);
L(2) = Link([0    0.383 0   pi/2    0   0]); %.1519 + .5174
L(3) = Link([0    0   -0.24365   0    0   0]);
L(4) = Link([0    0   -0.21325   0    0   0]);
L(5) = Link([0    0.11235     0   pi/2    0   0]);
L(6) = Link([0    0.08535     0   -pi/2    0   0]);
L(7) = Link([0    0.0819      0   0   0   0]);

L(1).qlim = [-2 2];
L(2).qlim = [-360 360]*pi/180;
L(3).qlim = [-360 360]*pi/180;
L(4).qlim = [-360 360]*pi/180;
L(5).qlim = [-360 360]*pi/180;
L(6).qlim = [-360 360]*pi/180;
L(7).qlim = [-360 360]*pi/180;
self.model = SerialLink(L);
q = zeros(1,self.model.n);
self.model.base = self.model.base * trotx(pi/2) * troty(pi/2);
self.model.plot(q,'workspace',[-1 1 -1 1 0 2]);
self.model.teach();

%%
PlaceObject("BeachRockFree_decimated.ply",[-8 8 0]);
