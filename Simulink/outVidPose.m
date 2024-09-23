function outCurrentPose = outVidPose(sv, currentPose)
    start_ok = isStateValid(sv, currentPose);
    d = 1; %metros
    x = currentPose(1);
    y = currentPose(2);
    ang = currentPose(3);
    xN = d*cos(ang);
    yN = d*sin(ang);

    while (~start_ok) 
        x_out = x + xN;
        y_out = y + yN;

        start_ok = isStateValid(sv, [x_out y_out ang]);

        x = x_out;
        y = y_out;
    end
    

    outCurrentPose = [x y ang];
end
