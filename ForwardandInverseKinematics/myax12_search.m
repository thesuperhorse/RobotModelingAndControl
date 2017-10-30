function myrobot = myax12_search(delta)
%Takes 4x4 DH table and returns a robot structure with perturbations on a3, a4, and d4 from the input delta
    DH = [
            pi/2 -55 0 142;
            0 172 0 0;
            pi/2 23 0 0;
            0 5 0 285];                                     %initial DH table
    
    DH(3,2) = DH(3,2) + delta(1);                           %perturbation on a3
    DH(4,2) = DH(4,2) + delta(2);                           %perturbation on a4
    DH(4,4) = DH(4,4) + delta(3);                           %perturbation on d4
   
    link_cell = cell(1,numel(DH(:,1)));                     %define empty cell to hold link variables
    for i = 1:numel(link_cell)                                   
        link_cell{i} = link([DH(i,:) 0], 'standard');       %fill in cell with links 1-6 according to DH table
    end
    
    myrobot = robot(link_cell);                             %make the robot

end