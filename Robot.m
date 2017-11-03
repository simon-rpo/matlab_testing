function Robot

clc;clear all;close all;

Tam = get(0,'ScreenSize');% Lee el tama�o de la pantalla y lo guarda en Tam
fig_1 = figure('Position',[0,35,Tam(3)-280,Tam(4)-150],...
    'Name',' Robot Brazo');% Genera una pantalla del tama?o se?alado en []
hold on;
light                               % Iluminacion por defecto
daspect([1 1 1])                    % ancho1, alto 1 profundidad 1
view(135,25)                        % Punto de vista azimut de 135� y elevaci�n de 25�
xlabel('X'),ylabel('Y'),zlabel('Z');% Muestra los ejes x,y,z
% title('ROBOT');% Pone un titulo como encabezado
posMas = 14;                  posMin = -posMas;
axis([posMin posMas posMin posMas posMin posMas]);% Genera los ejes con tama�os.
plot3([posMin,posMas],[posMin,posMin],[posMin,posMin],'k')
plot3([posMin,posMin],[posMin,posMas],[posMin,posMin],'k')
plot3([posMin,posMin],[posMin,posMin],[posMin,posMas],'k')
plot3([posMin,posMin],[posMas,posMas],[posMin,posMas],'k')
plot3([posMin,posMas],[posMin,posMin],[posMas,posMas],'k')
plot3([posMin,posMin],[posMin,posMas],[posMas,posMas],'k')
tamvec=100;
V=zeros(tamvec,4);caras=zeros(3*tamvec+4,3);
for t=0:1:tamvec;
    V(t+1,:)=[cos(t*pi/tamvec) sin(t*pi/tamvec) -0.5 1];
    V(tamvec+t+2,:)=[cos(t*pi/tamvec) sin(t*pi/tamvec) 0.5 1];
end

for i=1:1:tamvec-1
    caras(i,:)=[1,i+1,i+2];
    caras(tamvec-1+i,:)=[tamvec+2,i+2+tamvec,i+3+tamvec];
end

for i=1:1:tamvec
    caras(i+2*tamvec-2,:)=[i,i+1,tamvec+1+i];
    caras(3*tamvec-2+i,:)=[tamvec+1+i,tamvec+i+2,i+1];
end

caras(3*tamvec+3,:)=[1,tamvec+1,tamvec+2];

caras(3*tamvec+4,:)=[tamvec+2,tamvec+1,2*tamvec+2];


%**********************************************

tamvecbase=100;

Vbase=zeros(tamvecbase,4);carasbase=zeros(3*tamvecbase+4,3);

for t=0:1:tamvecbase;
    Vbase(t+1,:)=[2*cos(2*t*pi/tamvecbase) 2*sin(2*t*pi/tamvecbase) -2 1];
    Vbase(tamvecbase+t+2,:)=[2*cos(2*t*pi/tamvecbase) 2*sin(2*t*pi/tamvecbase) 0 1];
end


for i=1:1:tamvecbase-1
    carasbase(i,:)=[1,i+1,i+2];
    carasbase(tamvecbase-1+i,:)=[tamvecbase+2,i+2+tamvecbase,i+3+tamvecbase];
end

for i=1:1:tamvecbase
    carasbase(i+2*tamvecbase-2,:)=[i,i+1,tamvecbase+1+i];
    carasbase(3*tamvecbase-2+i,:)=[tamvecbase+1+i,tamvecbase+i+2,i+1];
end

%**********************************************

thetaz=0;thetay=0;thetax=-pi/2;

RZ=[cos(thetaz) sin(thetaz) 0 0;-sin(thetaz) cos(thetaz) 0 0;...
    0 0 1 0;0 0 0 1];

RY=[-sin(thetay) 0 cos(thetay) 0;0 1 0 0;cos(thetay) 0 sin(thetay) 0;...
    0 0 0 1];

RX=[1 0 0 0;0 cos(thetax) sin(thetax) 0;0 -sin(thetax) cos(thetax) 0;...
    0 0 0 1];

T=[1 0 0 0;0 1 0 1;0 0 1 0;0 0 0 1];

Q=RX*V';

V=Q';

VP1=V(:,1:3);

V_Base=Vbase(:,1:3);

p = patch('Faces',caras,'Vertices',VP1,'FaceColor','w');

p_base = patch('Faces',carasbase,'Vertices',V_Base,'FaceColor','w');

set(p,'facec', [0.9 0.4 0.05], 'EdgeColor','none','FaceColor','w');

set(p_base,'facec', [0.9 0.4 0.05], 'EdgeColor','none','FaceColor','w');


%pieza 2

tamvecp1=100;

V1=zeros(tamvecp1,4);carasp1=zeros(3*tamvecp1+4,3);

for t=0:1:tamvecp1;
    V1(t+1,:)=[cos(2*t*pi/tamvecp1) sin(2*t*pi/tamvecp1) 0 1];
    V1(tamvecp1+t+2,:)=[0.8*cos(2*t*pi/tamvecp1) 0.8*sin(2*t*pi/tamvecp1) 5 1];
end


for i=1:1:tamvecp1-1
    carasp1(i,:)=[1,i+1,i+2];
    carasp1(tamvecp1-1+i,:)=[tamvecp1+2,i+2+tamvecp1,i+3+tamvecp1];
end

for i=1:1:tamvecp1
    carasp1(i+2*tamvecp1-2,:)=[i,i+1,tamvecp1+1+i];
    carasp1(3*tamvecp1-2+i,:)=[tamvecp1+1+i,tamvecp1+i+2,i+1];
end

%*******************************************************************

tamvecp3=100;

VP3=zeros(tamvecp3,4);carasp3=zeros(3*tamvecp3+4,3);

for t=0:1:tamvecp3;
    VP3(t+1,:)=[0.9686*cos(t*pi/tamvecp3) 0.9686*sin(t*pi/tamvecp3) -0.2 1];
    VP3(tamvecp3+t+2,:)=[0.9686*cos(t*pi/tamvecp3) 0.9686*sin(t*pi/tamvecp3) 0.2 1];
end


for i=1:1:tamvecp3-1
    carasp3(i,:)=[1,i+1,i+2];
    carasp3(tamvecp3-1+i,:)=[tamvecp3+2,i+2+tamvecp3,i+3+tamvecp3];
end

for i=1:1:tamvecp3
    carasp3(i+2*tamvecp3-2,:)=[i,i+1,tamvecp3+1+i];
    carasp3(3*tamvecp3-2+i,:)=[tamvecp3+1+i,tamvecp3+i+2,i+1];
end

carasp3(3*tamvecp3+3,:)=[1,tamvecp3+1,tamvecp3+2];

carasp3(3*tamvecp3+4,:)=[tamvecp3+2,tamvecp3+1,2*tamvecp3+2];

%*********************************************************

%*******************************************************************

tamvecp4=100;

VP4=zeros(tamvecp4,4);carasp4=zeros(3*tamvecp4+4,3);

for t=0:1:tamvecp4;
    VP4(t+1,:)=[0.7*cos(t*pi/tamvecp4) 0.7*sin(t*pi/tamvecp4) -0.2 1];
    VP4(tamvecp4+t+2,:)=[0.82*cos(t*pi/tamvecp4) 0.82*sin(t*pi/tamvecp4) 0.2 1];
end


for i=1:1:tamvecp4-1
    carasp4(i,:)=[1,i+1,i+2];
    carasp4(tamvecp4-1+i,:)=[tamvecp4+2,i+2+tamvecp4,i+3+tamvecp4];
end

for i=1:1:tamvecp4

    carasp4(i+2*tamvecp4-2,:)=[i,i+1,tamvecp4+1+i];

    carasp4(3*tamvecp4-2+i,:)=[tamvecp4+1+i,tamvecp4+i+2,i+1];

end

carasp4(3*tamvecp4+3,:)=[1,tamvecp4+1,tamvecp4+2];

carasp4(3*tamvecp4+4,:)=[tamvecp4+2,tamvecp4+1,2*tamvecp4+2];

%*********************************************************

%*********************************************************

thetaz=0;thetay=0;thetax=pi/2;thetax1=-pi/2;

RZ=[cos(thetaz) sin(thetaz) 0 0;-sin(thetaz) cos(thetaz) 0 0;...
    0 0 1 0;0 0 0 1];

RY=[-sin(thetay) 0 cos(thetay) 0;0 1 0 0;cos(thetay) 0 sin(thetay) 0;...
    0 0 0 1];

RX=[1 0 0 0;0 cos(thetax) sin(thetax) 0.7;0 -sin(thetax) cos(thetax) 1;...
    0 0 0 1];

RXp4=[1 0 0 0;0 cos(thetax1) sin(thetax1) 0.9;0 -sin(thetax1) cos(thetax1) 6;...
    0 0 0 1];

T=[1 0 0 0;0 1 0 0.7;0 0 1 1;0 0 0 1];

Qp2=T*V1';

Qp3=RX*VP3';

Qp4=RXp4*VP4';

V1=Qp2';

%% 5092716


VP3=Qp3';

VP4=Qp4';

VP3_B=VP3(:,1:3);

VP4_B=VP4(:,1:3);

VP2=V1(:,1:3);%Tronco

p1 = patch('Faces',carasp1,'Vertices',VP2,'FaceColor','w');

p3 = patch('Faces',carasp3,'Vertices',VP3_B,'FaceColor','w');

p4 = patch('Faces',carasp4,'Vertices',VP4_B,'FaceColor','w');

set(p1,'facec', [0.9 0.4 0.05], 'EdgeColor','none','FaceColor','g');

set(p3,'facec', [0.9 0.4 0.05], 'EdgeColor','none','FaceColor','g');

set(p4,'facec', [0.9 0.4 0.05], 'EdgeColor','none','FaceColor','g');


%pieza 3

tamvecf1=100;

Vf1=zeros(tamvecf1,4);carasf1=zeros(3*tamvecf1+4,3);

for t=0:1:tamvecf1;

    Vf1(t+1,:)=[0.8*cos(2*t*pi/tamvecf1) 0.8*sin(2*t*pi/tamvecf1) 0 1];

    Vf1(tamvecf1+t+2,:)=[0.6*cos(2*t*pi/tamvecf1) 0.6*sin(2*t*pi/tamvecf1) 5 1];

end


for i=1:1:tamvecf1-1

    carasf1(i,:)=[1,i+1,i+2];

    carasf1(tamvecf1-1+i,:)=[tamvecf1+2,i+2+tamvecf1,i+3+tamvecf1];

end

for i=1:1:tamvecf1

    carasf1(i+2*tamvecf1-2,:)=[i,i+1,tamvecf1+1+i];

    carasf1(3*tamvecf1-2+i,:)=[tamvecf1+1+i,tamvecf1+i+2,i+1];

end

%*******************************************************************

tamvecf3=100;

Vf3=zeros(tamvecf3,4);carasf3=zeros(3*tamvecf3+4,3);

for t=0:1:tamvecf3;

    Vf3(t+1,:)=[0.71*cos(t*pi/tamvecf3) 0.71*sin(t*pi/tamvecf3) -0.2 1];

    Vf3(tamvecf3+t+2,:)=[0.8*cos(t*pi/tamvecf3) 0.8*sin(t*pi/tamvecf3) 0.2 1];

end


for i=1:1:tamvecf3-1

    carasf3(i,:)=[1,i+1,i+2];

    carasf3(tamvecf3-1+i,:)=[tamvecf3+2,i+2+tamvecf3,i+3+tamvecf3];

end

for i=1:1:tamvecf3

    carasf3(i+2*tamvecf3-2,:)=[i,i+1,tamvecf3+1+i];

    carasf3(3*tamvecf3-2+i,:)=[tamvecf3+1+i,tamvecf3+i+2,i+1];

end

carasf3(3*tamvecf3+3,:)=[1,tamvecf3+1,tamvecf3+2];

carasf3(3*tamvecf3+4,:)=[tamvecf3+2,tamvecf3+1,2*tamvecf3+2];

%*********************************************************

%*******************************************************************

tamvecf4=100;

Vf4=zeros(tamvecf4,4);carasf4=zeros(3*tamvecf4+4,3);

for t=0:1:tamvecf4;

    Vf4(t+1,:)=[0.3*cos(2*t*pi/tamvecf4) 0.3*sin(2*t*pi/tamvecf4) 0 1];

    Vf4(tamvecf4+t+2,:)=[0.3*cos(2*t*pi/tamvecf4) 0.3*sin(2*t*pi/tamvecf4) 1 1];

    

end


for i=1:1:tamvecf4-1

    carasf4(i,:)=[1,i+1,i+2];

    carasf4(tamvecf4-1+i,:)=[tamvecf4+2,i+2+tamvecf4,i+3+tamvecf4];

end

for i=1:1:tamvecf4

    carasf4(i+2*tamvecf4-2,:)=[i,i+1,tamvecf4+1+i];

    carasf4(3*tamvecf4-2+i,:)=[tamvecf4+1+i,tamvecf4+i+2,i+1];

end

carasf4(3*tamvecf4+3,:)=[1,tamvecf4+1,tamvecf4+2];

carasf4(3*tamvecf4+4,:)=[tamvecf4+2,tamvecf4+1,2*tamvecf4+2];

%*********************************************************

%*********************************************************

thetaz=0;thetay=0;thetax=pi/2;thetaxf1=0;

RZ=[cos(thetaz) sin(thetaz) 0 0;-sin(thetaz) cos(thetaz) 0 0;...
    0 0 1 0;0 0 0 1];

RY=[-sin(thetay) 0 cos(thetay) 0;0 1 0 0;cos(thetay) 0 sin(thetay) 0;...
    0 0 0 1];

RX=[1 0 0 0;0 cos(thetax) sin(thetax) 0.5;0 -sin(thetax) cos(thetax) 6.9;...
    0 0 0 1];

RXp4=[1 0 0 0;0 cos(thetaxf1) sin(thetaxf1) 0.6;0 -sin(thetaxf1) cos(thetaxf1) 11.9;...
    0 0 0 1];

Tp4=[1 0 0 0;0 1 0 0.6;0 0 1 6.9;0 0 0 1];

Qp2a=Tp4*Vf1';

Qp3a=RX*Vf3';

Qp4a=RXp4*Vf4';

Vf1=Qp2a';

Vf3=Qp3a';

Vf4=Qp4a';

VP3_Bf=Vf3(:,1:3);

VP4_Bf=Vf4(:,1:3);

VP2f=Vf1(:,1:3);

p1a = patch('Faces',carasf1,'Vertices',VP2f,'FaceColor','w');

p3a = patch('Faces',carasf3,'Vertices',VP3_Bf,'FaceColor','w');

p4a = patch('Faces',carasf4,'Vertices',VP4_Bf,'FaceColor','w');

set(p1a,'facec', [0.9 0.4 0.05], 'EdgeColor','none', 'FaceColor', 'b');

set(p3a,'facec', [0.9 0.4 0.05], 'EdgeColor','none', 'FaceColor', 'b');

set(p4a,'facec', [0.9 0.4 0.05], 'EdgeColor','none', 'FaceColor', 'b');


%%movimiento %%

%%primera articulacion 

xPan = 20;          yPan = 20;          ancPan = 200;       altPan = 120;

xIni = xPan + 20;                       yIni = yPan + 50;

ancSli = 120;                           altSli = 20;

AngAntZ=0;  AngAntX1=0;   AngAntX2=0;

panelEnt = uipanel(fig_1, 'units','pixels', 'Position',...
    [xPan yPan ancPan altPan], 'Title','Angulos','FontSize',9);% Construye un panel en la pare inferior

t1_slider = uicontrol(panelEnt,'style','slider',...
    'Max',90,'Min',-90,'Value',0, 'SliderStep',[0.05 0.2],...
    'callback',@sliderAng1, 'Position',[xIni yIni ancSli altSli]);

t2_slider = uicontrol(panelEnt,'style','slider',...
    'Max',135,'Min',45,'Value',90, 'SliderStep',[0.05 0.2],...
    'callback',@sliderAng2, 'Position',[xIni yIni-25 ancSli altSli]);

t3_slider = uicontrol(panelEnt,'style','slider',...
    'Max',135,'Min',45,'Value',90, 'SliderStep',[0.05 0.2],...
    'callback',@sliderAng3, 'Position',[xIni yIni-50 ancSli altSli]);

Todos=[V;VP3;V1;VP4;Vf3;Vf1;Vf4];%Todo el sistema sin la base






ang_z=0;ang_y1=pi/2;ang_y2=pi/2;


function sliderAng1(mane, evento)

ang_z = round(get(mane,'Value'))*pi/180;

BandAng_1=1;

dinaDire(ang_z,BandAng_1)

end

function sliderAng2(mane, evento)

ang_y1 = round(get(mane,'Value'))*pi/180;

BandAng_2=2;

dinaDire(ang_y1,BandAng_2)

  

end

function sliderAng3(mane, evento)

ang_y2 = round(get(mane,'Value'))*pi/180;

BandAng_3=3;

dinaDire(ang_y2,BandAng_3)

end


function dinaDire(valDesli,BandAng)

Todo=Todos;%Todo el sistema sin la base

RZg=[cos(ang_z) -sin(ang_z) 0 0;sin(ang_z) cos(ang_z) 0 0;...
    0 0 1 0;0 0 0 1];

GTodoz=RZg*Todo';

Todo=GTodoz';

RYg=[-sin(ang_y1) 0 cos(ang_y1) 0;0 1 0 0.7;cos(ang_y1) 0 sin(ang_y1) 0.0314;...
    0 0 0 1];

RZg_inv_Ante_bra=[cos(ang_z) sin(ang_z) 0 0;-sin(ang_z) cos(ang_z) 0 -0.7;...
    0 0 1 -0.0314;0 0 0 1];

% RXg=[1 0 0 0;0 cos(ang_x1) -sin(ang_x1) 0.7;0 sin(ang_x1) cos(ang_x1) 0.0314;...
%     0 0 0 1];

% RXg2=[1 0 0 0;0 cos(ang_x2) -sin(ang_x2) 0.9;0 sin(ang_x2) cos(ang_x2) 6;...
%     0 0 0 1];

AnteBra_Bra=Todo(203:1414,:);%Antebrazo y brazo

GAntebra=RZg*RYg*RZg_inv_Ante_bra*AnteBra_Bra';

AnteBra_Bra=GAntebra';

RZg_inv_bra=[cos(ang_z) sin(ang_z) 0 0;-sin(ang_z) cos(ang_z) 0 0;...
    0 0 1 0;0 0 0 1];

RYg_inv_bra=[-sin(ang_y1) 0 cos(ang_y1) 0;0 1 0 -0.5;cos(ang_y1) 0 sin(ang_y1) -6.545;...
    0 0 0 1];

RYg2=[-sin(ang_y2) 0 cos(ang_y2) 0;0 1 0 0.5;cos(ang_y2) 0 sin(ang_y2) 6.545;...
    0 0 0 1];

RYg1=[-sin(ang_y1) 0 cos(ang_y1) 0;0 1 0 0;cos(ang_y1) 0 sin(ang_y1) 0;...
    0 0 0 1];

Brazo=AnteBra_Bra(607:1212,:);%brazo

GBrazo=RZg*RYg1*RYg2*RYg_inv_bra*RZg_inv_bra*Brazo';

Brazo=GBrazo';

V=Todo(1:202,:);VP3=AnteBra_Bra(1:202,:);V1=AnteBra_Bra(203:404,:);

VP4=AnteBra_Bra(405:606,:);Vf3=Brazo(1:202,:);Vf1=Brazo(203:404,:);Vf4=Brazo(405:606,:);

set(p,'vertices', V(:,1:3));

set(p3,'vertices', VP3(:,1:3));

set(p1,'vertices', V1(:,1:3));

set(p4,'vertices', VP4(:,1:3));

set(p3a,'vertices', Vf3(:,1:3));

set(p1a,'vertices', Vf1(:,1:3));

set(p4a,'vertices', Vf4(:,1:3));

pause(2)

drawnow


end






end






