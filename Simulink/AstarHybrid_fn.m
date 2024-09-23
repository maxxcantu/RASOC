function waypoints = AstarHybrid_fn(currentPose, goalPose)
    %% Lectura de imagen seccionada con Image Labeler
    % image = imread('vidHileras1_5\Label_1.png');
    % map = binaryOccupancyMap(image,30);

    resolution = 16.4;
    image = imread('FincaNueva_Label\Label_1_finca_nueva.png');
    map = binaryOccupancyMap(image, resolution);
%     show(map);

    %% Generar mapa de costos
    % Dimensiones y restricciones geometricas del vehiculo
    % vehicleDims = vehicleDimensions(0.4, 0.4, 'FrontOverhang',0.1,'RearOverhang',0.1,'Wheelbase',0.2);
    % vehicleDims = vehicleDimensions(1.2, 0.8, 'FrontOverhang',0.3,'RearOverhang',0.3,'Wheelbase',0.6);
    vehicleDims = vehicleDimensions(1.4, 1, 'FrontOverhang',0.3,'RearOverhang',0.3,'Wheelbase',0.6);
    % ccConfig = inflationCollisionChecker(vehicleDims,3);
    ccConfig = inflationCollisionChecker(vehicleDims,1);

    % Mapa de costos con restricciones
    costmap = vehicleCostmap(map, 'CollisionChecker', ccConfig);
    plot(costmap);

    %% Pose inicial y final - [x y theta]
%     currentPose = [4 5 pi/2];
%     goalPose = [47 43.5 0];
    %figure; show(map);

    %% Validador de mapa de ocupación - mapa blanco y negro
    % ss = stateSpaceSE2;
    % ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; -pi pi];
    % sv = validatorOccupancyMap(ss);
    % sv.Map = map;

    %% Validador de mapa de costos - mapa con rojo y rosadito
    ss = stateSpaceSE2;
    sv = validatorVehicleCostmap(ss);
    sv.Map = costmap;

    %% Cambio punto inicio
    outCurrentPose = outVidPose(sv, currentPose);

    %% Planificador de trayectoria global A* Híbrido
    planner = plannerHybridAStar(sv);
    planner.InterpolationDistance = 3;
%     planner.DirectionSwitchingCost = 3;
    planner.MinTurningRadius = 1;
    pathSimple = plan(planner, outCurrentPose, goalPose);

    figure; show(planner);

    %% Waypoints de trayectoria global - [x y]
    waypoints = [currentPose(1), currentPose(2);
                 pathSimple.States(:,1), pathSimple.States(:,2)];
              
%     resolution = 16.4;
%     image = imread('FincaNueva_Label\Label_1_finca_nueva.png');
%     map = binaryOccupancyMap(image, resolution);
%     show(map);
%   figure; plot(costmap); hold on; plot(waypoints(:,1),waypoints(:,2));
%   figure; plot(costmap); hold on; plot(waypoints(:,1),waypoints(:,2),'-ob',"LineWidth",2,"DisplayName",'Path');
%   figure; show(map); hold on; plot(waypoints(:,1),waypoints(:,2),'-ob',"LineWidth",2,"DisplayName",'Path');



end