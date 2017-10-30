function myrobot = mypuma560(DH)
%Takes 6x4 DH table and returns a robot structure

    link_cell = cell(1,6);                                  %define empty cell to hold link variables
    for i = 1:6                                     
        link_cell{i} = link([DH(i,:) 0], 'standard');       %fill in cell with links 1-6 according to DH table
    end
    
    myrobot = robot(link_cell);                             %make the robot

end