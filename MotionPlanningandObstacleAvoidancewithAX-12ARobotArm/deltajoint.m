function q_error = deltajoint(delta)
    
    myrobot = myax12_search(delta);

    Q1 = [ 
           -0.4500
            0.5216
           -0.5420
            1.4471];      %measured joint angles for position X1

    Q2 = [  0.5931
            0.3170
                 0
            1.5340];      %measured joint angles for position X2
        
    Q3 = [  1.2476
            0.3784
           -0.1125
            1.5340];      %measured joint angles for position X3

        
    X1 = [10, -5, 0]*13;
    X2 = [15, 10, 0]*13;
    X3 = [5, 15, 0]*13;



    
    q_error = norm(Q1-inverse_ax12(myrobot, X1)) + norm(Q2 - inverse_ax12(myrobot, X2)) + norm(Q3-inverse_ax12(myrobot, X3));

end