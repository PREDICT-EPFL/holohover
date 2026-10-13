file_name = "circle_large.yaml";
fileID = fopen(file_name,'w');
fprintf(fileID, "time_step: %f\n", 0.5);
fprintf(fileID, "\n");
fprintf(fileID, "names: [o4]\n");
fprintf(fileID, "ids: [4]\n");
fprintf(fileID, "\n");
fprintf(fileID, "trajectory:\n");

for k = 1:5   
    for i = 1:16
        r = 0.3;
        theta = 2*pi/16*i;
        x(i) = r*cos(-theta);
        y(i) = r*sin(-theta);
        fprintf(fileID, "  - step:\n");
        fprintf(fileID, "      - id: 4\n");
        fprintf(fileID, "        x: %f\n",x(i));
        fprintf(fileID, "        y: %f\n",y(i));
        fprintf(fileID, "        yaw: 0.0\n");
    end


end