setgripper(m,75)
pause(0.5)
for i = 1:numel(path0(:,1))
    
   setangles(m,path0(i,:));    
    
   pause(0.02) 
end

for i = 1:numel(path1(:,1))
    
   setangles(m,path1(i,:));    
    
   pause(0.02) 
end

pause(1)
setgripper(m,0)
pause(1)
setangles(m,[q_cup(1:3); pi]);
pause(1)
setangles(m,q_pick1)
pause(1)
setgripper(m,1)

pause(0.5)

for i = 1:numel(path2(:,1))
    
   setangles(m,path2(i,:));    
    
   pause(0.02) 
end

for i = 1:numel(path3(:,1))
    
   setangles(m,path3(i,:));    
    
   pause(0.02) 
end

 setangles(m, [q_drop(1:3); pi/2])