close all
clear all
clc

%% Posición Inicial y Final para Trayectoria
numRuta = 1; %Rutas 1, 2 o 3
calc_planner = 0; %0: NO calcular Planner, 1: SI recalcular Planner

if (numRuta == 1) % Ruta 1
    currentPose = [4 5 pi/2];
    goalPose = [47 43.5 0];
    cargado = 1; % Ida con carga
elseif (numRuta == 2) % Ruta 2
    currentPose = [4 5 -pi/2];
    goalPose = [47 43.5 0];
    cargado = 1; % Ida con carga
elseif (numRuta == 3) % Ruta 3
    currentPose = [47 43.5 0];
    goalPose = [1.5 5 pi/2];
    cargado = 0; % Vuelta sin carga
end

gpsBasePose = [1 44];
%% PID - Usamos solo PI porque la D produce Inestabilidades
% P = 56.2149208018988;
% I = 55.3182760647861;

%% Modulador de Tensión
Vnom = 310;                  %%% [V] Tensión Nominal de los Motores
t_lpf = 1/200;              %%% [s] Constante de Tiempo del Filtro PB del Modulador.

%% Características de los Motores BLDC (BSG23-28AA-02)
kt = 0.077*sqrt(2)*3;       %%% [Nm/A] Constante de Torque
kb = 8.93*(3/(1000*pi));    %%% [V/rad/s] Voltaje Contra Electromotriz
Ipeak = 21.7;               %%% [A] Corriente Pico del Motor
Ra = Vnom/Ipeak;            %%% [Ohm] Resistencia del Devanado

%% Desgaste y Fricción GENERAL
B = 0.5;            %%% [Nms] Coeficiente de Fricción Viscosa [Newton*metro*segundo]

%% Modulador de Torque 
tau_max = 5;          %%% [Nm] Torque Máximo, es una protección.
bm = 4.8 * (1e-7);      %%% [kg.m2] Inercia del Rotor.

%% Modulador de Corriente
Rp = Ra*20;             %%% [Ohm] Ganancia Ley Proporcional

%% Caja Reductora
n = 16;
efic_red = 0.95;

%% Geometría del Robot
a = 0.445;      %%% Distancia del Eje Motriz al Centro de Masa
L = 0.55;       %%% Separación entre ruedas
r = 0.125;      %%% Radio de Rueda Motriz
length = 1.25;
width = 0.75;
% height = 0.79;

%% Dinámica del Robot
M = 80;
mg = 1.5;
mc = 1;
rc = 0.08;
I = M*(length^2 + width^2)/12;
if (cargado)
    m_uva = 20; % Kg - masa de cajon de uva lleno
else
    m_uva = 0; % Kg - masa de cajon de uva vacio
end


%% Cinemática del Robot
dt = 1;
v_max = 0.8; %m/s
% a_max = 0.5; %m/s^2 
w_max = 0.5; %rad/s
% alfa_max = 0.5; %rad/s^2
R = v_max/w_max; %Radio de giro - m

a_max = v_max/dt; %m/s^2
alfa_max = w_max/dt; %rad/s^2

lidar_dist = 0.7; %m - Lidar
alt_z = 0; %m - Altura en Z para calcular X e Y de GPS

%% MAPA BINARIO DESDE IMAGEN DE DRONE DE VIÑEDO CON IMAGE LABELER

if (calc_planner == 0)
    if (numRuta == 1)
        load 'C:\Users\maxim\OneDrive\Documentos\1. Ingenieria en Mecatronica\43. PROYECTO FINAL DE ESTUDIOS\Tesis\Simulink_ROS\Simulink\WP_ruta1_fincaNueva_out.mat';
    elseif (numRuta == 2)
        load 'C:\Users\maxim\OneDrive\Documentos\1. Ingenieria en Mecatronica\43. PROYECTO FINAL DE ESTUDIOS\Tesis\Simulink_ROS\Simulink\WP_ruta2_fincaNueva_out.mat';
    elseif (numRuta == 3)
        load 'C:\Users\maxim\OneDrive\Documentos\1. Ingenieria en Mecatronica\43. PROYECTO FINAL DE ESTUDIOS\Tesis\Simulink_ROS\Simulink\WP_ruta3_fincaNueva_out.mat';
    end
else
    waypoints = AstarHybrid_fn(currentPose, goalPose);
end



%% Distancia recorrida total
dist_rec = 0;
Dx = 0;
Dy = 0;
[wp, l] = size(waypoints);
for w=1:wp-1
    Dx = waypoints(w+1,1) - waypoints(w,1);
    Dy = waypoints(w+1,2) - waypoints(w,2);
    aux_dist = sqrt((Dx^2)+(Dy^2));
    dist_rec = dist_rec + aux_dist;
end

%% Plot trayectoria completa
resolution = 16.4;
image = imread('FincaNueva_Label\Label_1_finca_nueva.png');
map = binaryOccupancyMap(image, resolution);
figure; show(map);
hold all;
plot(waypoints(:,1),waypoints(:,2),'-ob',"LineWidth",2,"DisplayName",'Path');
plot(currentPose(1),currentPose(2),'xg',"LineWidth",5,"DisplayName","Start");
plot(goalPose(1),goalPose(2),'xr',"LineWidth",5,"DisplayName","End");
hold off;


%%% Filtrado de imagen sin Image Labeler
% finca_nueva = imread('C:\Users\maxim\OneDrive\Documentos\1. Ingenieria en Mecatronica\43. PROYECTO FINAL DE ESTUDIOS\Tesis\Simulink_ROS\Imágenes y Videos\finca_nueva.png')
% imgGris = rgb2gray(finca_nueva);
% imshow(imgGris);
% BW = imbinarize(imgGris);
% imshow(BW);

% setenv('ROS_DOMAIN_ID','25')
% ros2 topic list
