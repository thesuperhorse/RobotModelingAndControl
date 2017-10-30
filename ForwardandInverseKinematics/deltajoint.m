function q_error = deltajoint(delta)
% Calculate the error between the inverse kinematics output and 3 measured positions    
    
    
    myrobot = myax12_search(delta);

    Q1 = [ 0.6750    0.2148    0.2812    1.5647 ]';     %measured joint angles for position X1

    Q2 = [ 0.8437    0.2454    0.2148    1.5647]';      %measured joint angles for position X2
        
    Q3 = [ 1.4471    0.4960   -0.4858    1.5544]';      %measured joint angles for position X3

        
    X1 = [17,  15.5, 0]*13;
    X2 = [14, 17.5, 0]*13;
    X3 = [1, 13.5, 0]*13;



    
    q_error = norm(Q1-inverse_ax12(myrobot, X1)) + norm(Q2 - inverse_ax12(myrobot, X2)) + norm(Q3-inverse_ax12(myrobot, X3));

end