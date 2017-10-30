function myrobot = myax12(DH)
    %Takes 4x4 DH table and returns a robot structure
   delta = [ 18.5770  109.4362   93.2059  158.8121];

    DH(3,2) = DH(3,2) + delta(1);
    DH(4,2) = DH(4,2) + delta(2);
    DH(4,4) = DH(4,4) + delta(3);
    DH(1,4) = DH(1,4) + delta(4);
   

    link_cell = cell(1,numel(DH(:,1)));                                  %define empty cell to hold link variables
    for i = 1:numel(link_cell)                                   
        link_cell{i} = link([DH(i,:) 0], 'standard');       %fill in cell with links 1-6 according to DH table
    end
    
    myrobot = robot(link_cell);                             %make the robot

end